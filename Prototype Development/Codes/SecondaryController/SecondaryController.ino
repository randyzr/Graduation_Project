#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "EspNowProtocol.h"

// =============================================================================
// SECONDARY CONTROLLER: HZR-171 TESTER + GM67 BADGE READER
// =============================================================================
// This ESP32 reads four digital signals from the HZR-171:
//   LOW, HIGH, PASS/OK, and TEST_STARTED/BP.
// It removes short electrical glitches with a 30 ms debounce, sends the stable
// values to the MainController, and operates the GM67 only when requested.

// ----- Hardware pins and timing settings -------------------------------------

constexpr uint8_t WRIST_LOW_PIN = 27;
constexpr uint8_t WRIST_HIGH_PIN = 14;
constexpr uint8_t WRIST_PASS_PIN = 12;
constexpr uint8_t TEST_STARTED_PIN = 13;
constexpr uint8_t BARCODE_RX_PIN = 16;  // ESP32 RX2
constexpr uint8_t BARCODE_TX_PIN = 17;  // ESP32 TX2
constexpr uint32_t BARCODE_BAUD = 9600;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
constexpr uint32_t INPUT_DEBOUNCE_MS = 30;
constexpr uint32_t BADGE_SCAN_TIMEOUT_MS = 10000;
constexpr uint32_t GM67_DECODE_REFRESH_MS = 4000;

const uint8_t GM67_HOST_MODE[] = {0x07, 0xC6, 0x04, 0x08, 0x00,
                                  0x8A, 0x08, 0xFE, 0x95};
const uint8_t GM67_START_DECODE[] = {0x04, 0xE4, 0x04, 0x00, 0xFF, 0x14};
const uint8_t GM67_STOP_DECODE[] = {0x04, 0xE5, 0x04, 0x00, 0xFF, 0x13};
const uint8_t GM67_LED_ON[] = {0x05, 0xE7, 0x04, 0x00, 0x01, 0xFF, 0x0F};
const uint8_t GM67_LED_OFF[] = {0x05, 0xE8, 0x04, 0x00, 0x01, 0xFF, 0x0E};
const uint8_t GM67_SCAN_ENABLE[] = {0x04, 0xE9, 0x04, 0x00, 0xFF, 0x0F};

// ----- Shared runtime state --------------------------------------------------
// stableFlags is the trusted HZR-171 sample. lastRawFlags is the newest sample
// before it has remained unchanged long enough to pass the debounce filter.

const uint8_t BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

EsdNowMessage statusMessage{};
String barcodeBuffer;
uint32_t sequenceNumber = 0;
uint32_t lastSendMs = 0;
uint32_t lastInputChangeMs = 0;
uint8_t lastRawFlags = 0;
uint8_t stableFlags = 0;
volatile bool badgeCommandPending = false;
volatile uint32_t lastBadgeCommandSequence = 0;
bool badgeScanArmed = false;
uint32_t badgeScanStartedMs = 0;
uint32_t lastDecodeStartMs = 0;

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data,
                     int length) {
  if (length != sizeof(EsdNowMessage)) return;
  EsdNowMessage command{};
  memcpy(&command, data, sizeof(command));
  if (command.magic == ESD_MESSAGE_MAGIC &&
      command.version == ESD_PROTOCOL_VERSION &&
      command.type == MESSAGE_READ_BADGE &&
      command.sequence != lastBadgeCommandSequence) {
    lastBadgeCommandSequence = command.sequence;
    badgeCommandPending = true;
  }
}

// ----- GM67 command and HZR-171 input helpers --------------------------------

void sendGm67Command(const uint8_t *command, size_t length) {
  Serial2.write(command, length);
  Serial2.flush();
  delay(80);
  // Discard the binary ACK/NAK response so it is never interpreted as badge
  // text (the ACK contains a printable 0x28 byte).
  while (Serial2.available()) Serial2.read();
}

uint8_t readInputFlags() {
  // Pack four GPIO values into one byte so they can be compared together.
  return (digitalRead(WRIST_LOW_PIN) ? 0x01 : 0) |
         (digitalRead(WRIST_HIGH_PIN) ? 0x02 : 0) |
         (digitalRead(WRIST_PASS_PIN) ? 0x04 : 0) |
         (digitalRead(TEST_STARTED_PIN) ? 0x08 : 0);
}

// ----- ESP-NOW status transmission ------------------------------------------

bool initializeEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(ESD_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Failed to set ESP-NOW channel.");
    return false;
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("Failed to initialize ESP-NOW.");
    return false;
  }
  esp_now_register_recv_cb(onEspNowReceive);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BROADCAST_ADDRESS, 6);
  peer.channel = ESD_ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW broadcast peer.");
    return false;
  }

  Serial.print("WROOM MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("ESP-NOW ready on channel %u.\n", ESD_ESPNOW_CHANNEL);
  return true;
}

void sendStatus() {
  statusMessage.magic = ESD_MESSAGE_MAGIC;
  statusMessage.version = ESD_PROTOCOL_VERSION;
  statusMessage.source = NODE_WROOM;
  statusMessage.type = MESSAGE_WROOM_STATUS;
  statusMessage.sequence = ++sequenceNumber;
  statusMessage.uptimeMs = millis();
  statusMessage.wristLow = (stableFlags & 0x01) != 0;
  statusMessage.wristHigh = (stableFlags & 0x02) != 0;
  statusMessage.wristPass = (stableFlags & 0x04) != 0;
  statusMessage.testStarted = (stableFlags & 0x08) != 0;
  statusMessage.badgeReaderActive = badgeScanArmed;

  const esp_err_t result = esp_now_send(BROADCAST_ADDRESS,
      reinterpret_cast<const uint8_t *>(&statusMessage), sizeof(statusMessage));
  Serial.printf("ESP-NOW WROOM packet #%lu: %s\n",
                static_cast<unsigned long>(sequenceNumber),
                result == ESP_OK ? "queued" : "ERROR");
  lastSendMs = millis();
}

// ----- One-shot badge-reading session ---------------------------------------

void finishBarcode() {
  barcodeBuffer.trim();
  // Some GM67 firmware revisions emit a comma for each accepted host command
  // immediately before the decoded text. Remove only that transport prefix;
  // commas inside the actual badge value remain untouched.
  while (barcodeBuffer.startsWith(",")) barcodeBuffer.remove(0, 1);
  barcodeBuffer.trim();
  if (barcodeBuffer.isEmpty()) return;

  barcodeBuffer.toCharArray(statusMessage.barcode,
                            sizeof(statusMessage.barcode));
  Serial.print("Barcode: ");
  Serial.println(statusMessage.barcode);
  barcodeBuffer = "";
  badgeScanArmed = false;
  sendGm67Command(GM67_STOP_DECODE, sizeof(GM67_STOP_DECODE));
  sendGm67Command(GM67_LED_OFF, sizeof(GM67_LED_OFF));
  sendStatus();
}

void readBarcodeReader() {
  while (Serial2.available()) {
    const char character = static_cast<char>(Serial2.read());
    if (!badgeScanArmed) continue;
    if (character == '\r' || character == '\n') {
      finishBarcode();
    } else if (isPrintable(character) &&
               barcodeBuffer.length() < sizeof(statusMessage.barcode) - 1) {
      barcodeBuffer += character;
    }
  }
}

void startOneBadgeRead() {
  badgeCommandPending = false;
  barcodeBuffer = "";
  memset(statusMessage.barcode, 0, sizeof(statusMessage.barcode));
  while (Serial2.available()) Serial2.read();
  badgeScanArmed = true;
  badgeScanStartedMs = millis();
  // Reassert host control in case the module reset, then explicitly enable
  // scanning and illumination before beginning the one-shot decode.
  sendGm67Command(GM67_HOST_MODE, sizeof(GM67_HOST_MODE));
  sendGm67Command(GM67_SCAN_ENABLE, sizeof(GM67_SCAN_ENABLE));
  sendGm67Command(GM67_LED_ON, sizeof(GM67_LED_ON));
  sendGm67Command(GM67_START_DECODE, sizeof(GM67_START_DECODE));
  lastDecodeStartMs = badgeScanStartedMs;
  Serial.println("GM67 one-shot badge read started.");
  sendStatus();
}

void stopBadgeReadOnTimeout() {
  badgeScanArmed = false;
  barcodeBuffer = "";
  sendGm67Command(GM67_STOP_DECODE, sizeof(GM67_STOP_DECODE));
  sendGm67Command(GM67_LED_OFF, sizeof(GM67_LED_OFF));
  Serial.println("GM67 badge read timed out.");
  sendStatus();
}

void keepBadgeReaderActive(uint32_t now) {
  if (!badgeScanArmed || now - lastDecodeStartMs < GM67_DECODE_REFRESH_MS) {
    return;
  }

  // The GM67 ends a single decode command after about five seconds. Restart
  // decoding before that internal limit so the aimer and illumination remain
  // active for the complete ten-second badge session.
  sendGm67Command(GM67_LED_ON, sizeof(GM67_LED_ON));
  sendGm67Command(GM67_START_DECODE, sizeof(GM67_START_DECODE));
  lastDecodeStartMs = now;
}

// ----- Arduino entry points --------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial2.begin(BARCODE_BAUD, SERIAL_8N1, BARCODE_RX_PIN, BARCODE_TX_PIN);
  delay(300);
  sendGm67Command(GM67_HOST_MODE, sizeof(GM67_HOST_MODE));
  sendGm67Command(GM67_STOP_DECODE, sizeof(GM67_STOP_DECODE));
  sendGm67Command(GM67_LED_OFF, sizeof(GM67_LED_OFF));
  Serial.println("GM67 configured for host-triggered one-shot reading.");

  // The ESD tester signals must be 0-3.3 V. Never apply 5 V to ESP32 GPIO.
  pinMode(WRIST_LOW_PIN, INPUT_PULLDOWN);
  pinMode(WRIST_HIGH_PIN, INPUT_PULLDOWN);
  pinMode(WRIST_PASS_PIN, INPUT_PULLDOWN);
  pinMode(TEST_STARTED_PIN, INPUT_PULLDOWN);

  stableFlags = lastRawFlags = readInputFlags();
  Serial.printf("Tester GPIO initial: LOW=%u HIGH=%u PASS=%u START=%u\n",
                digitalRead(WRIST_LOW_PIN), digitalRead(WRIST_HIGH_PIN),
                digitalRead(WRIST_PASS_PIN), digitalRead(TEST_STARTED_PIN));
  initializeEspNow();
  sendStatus();
}

void loop() {
  readBarcodeReader();

  if (badgeCommandPending) startOneBadgeRead();
  const uint32_t now = millis();
  if (badgeScanArmed && now - badgeScanStartedMs >= BADGE_SCAN_TIMEOUT_MS) {
    stopBadgeReadOnTimeout();
  } else {
    keepBadgeReaderActive(now);
  }

  // A change becomes trusted only after every signal stays stable for 30 ms.
  const uint8_t rawFlags = readInputFlags();
  if (rawFlags != lastRawFlags) {
    lastRawFlags = rawFlags;
    lastInputChangeMs = now;
    Serial.printf("Tester GPIO change: LOW=%u HIGH=%u PASS=%u START=%u\n",
                  digitalRead(WRIST_LOW_PIN), digitalRead(WRIST_HIGH_PIN),
                  digitalRead(WRIST_PASS_PIN),
                  digitalRead(TEST_STARTED_PIN));
  }
  if (rawFlags != stableFlags && now - lastInputChangeMs >= INPUT_DEBOUNCE_MS) {
    stableFlags = rawFlags;
    sendStatus();
  }
  if (now - lastSendMs >= HEARTBEAT_INTERVAL_MS) {
    sendStatus();
  }
  delay(2);
}
