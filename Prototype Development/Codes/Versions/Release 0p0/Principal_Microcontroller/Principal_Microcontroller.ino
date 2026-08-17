#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <ESP_I2S.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "Elecrow_7inch_HMI.h"
#include "EspNowProtocol.h"

// I2C wiring currently used by the principal ESP32-S3.
constexpr uint8_t I2C_SDA_PIN = 19;
constexpr uint8_t I2C_SCL_PIN = 20;

// Elecrow CrowPanel 7-inch HMI onboard speaker (I2S amplifier).
constexpr int8_t SPEAKER_I2S_BCLK_PIN = 42;
constexpr int8_t SPEAKER_I2S_LRCLK_PIN = 18;
constexpr int8_t SPEAKER_I2S_DATA_PIN = 17;
constexpr uint32_t SPEAKER_SAMPLE_RATE = 16000;
constexpr int16_t SPEAKER_AMPLITUDE = 3500;

constexpr int16_t PLAY_BUTTON_X = 50;
constexpr int16_t PLAY_BUTTON_Y = 180;
constexpr int16_t PLAY_BUTTON_WIDTH = 300;
constexpr int16_t PLAY_BUTTON_HEIGHT = 100;
constexpr int16_t REGISTER_BUTTON_X = 50;
constexpr int16_t REGISTER_BUTTON_Y = 180;
constexpr int16_t REGISTER_BUTTON_WIDTH = 300;
constexpr int16_t REGISTER_BUTTON_HEIGHT = 100;
constexpr int16_t BADGE_BUTTON_X = 450;
constexpr int16_t BADGE_BUTTON_Y = 180;
constexpr int16_t BADGE_BUTTON_WIDTH = 300;
constexpr int16_t BADGE_BUTTON_HEIGHT = 100;

constexpr uint8_t SHT31_ADDRESS_PRIMARY = 0x44;
constexpr uint8_t SHT31_ADDRESS_SECONDARY = 0x45;
constexpr uint8_t GT911_ADDRESS_PRIMARY = 0x14;
constexpr uint8_t GT911_ADDRESS_SECONDARY = 0x5D;
constexpr uint16_t GT911_STATUS_REGISTER = 0x814E;
constexpr uint16_t GT911_FIRST_POINT_REGISTER = 0x814F;
constexpr unsigned long READ_INTERVAL_MS = 2000;
constexpr unsigned long SENSOR_RETRY_INTERVAL_MS = 5000;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
I2SClass speakerI2s;
Elecrow7InchHMI display;

bool sht31Detected = false;
uint8_t activeSht31Address = 0;
unsigned long lastReadTime = 0;
unsigned long lastRetryTime = 0;
float lastTemperatureC = NAN;
float lastRelativeHumidity = NAN;
bool speakerReady = false;
bool touchWasDown = false;
uint8_t activeGt911Address = GT911_ADDRESS_PRIMARY;
portMUX_TYPE espNowMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool espNowPacketPending = false;
EsdNowMessage pendingEspNowMessage{};
EsdNowMessage wroomStatus{};
EsdNowMessage cameraStatus{};
unsigned long lastWroomPacketMs = 0;
unsigned long lastCameraPacketMs = 0;
int16_t lastEnrolledId = -1;
bool enrollmentRequestedFromHmi = false;
uint32_t commandSequence = 0;
bool badgeRequestPending = false;
const uint8_t BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void playMelody();

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data,
                     int length) {
  if (length != sizeof(EsdNowMessage)) return;

  EsdNowMessage incoming{};
  memcpy(&incoming, data, sizeof(incoming));
  if (incoming.magic != ESD_MESSAGE_MAGIC ||
      incoming.version != ESD_PROTOCOL_VERSION) return;

  portENTER_CRITICAL(&espNowMux);
  pendingEspNowMessage = incoming;
  espNowPacketPending = true;
  portEXIT_CRITICAL(&espNowMux);
}

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
  Serial.print("Principal HMI MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("ESP-NOW receiver ready on channel %u.\n", ESD_ESPNOW_CHANNEL);
  return true;
}

void drawRemoteStatus() {
  const bool wroomOnline = millis() - lastWroomPacketMs < 3000;
  const bool cameraOnline = millis() - lastCameraPacketMs < 3000;

  display.fillRect(0, 290, 800, 65, 0x0841);
  display.setTextDatum(middle_center);
  display.setTextSize(2);

  char esdText[110];
  if (!wroomOnline) {
    snprintf(esdText, sizeof(esdText), "ESD tester: OFFLINE");
    display.setTextColor(TFT_ORANGE, 0x0841);
  } else {
    const char *result = wroomStatus.wristPass ? "PASS" :
                         (wroomStatus.wristLow ? "FAIL LOW" :
                          (wroomStatus.wristHigh ? "FAIL HIGH" : "WAITING"));
    if (wroomStatus.badgeReaderActive || badgeRequestPending) {
      snprintf(esdText, sizeof(esdText), "ESD: %s   READING BADGE...", result);
    } else {
      snprintf(esdText, sizeof(esdText), "ESD: %s   Test:%s   Badge: %s", result,
               wroomStatus.testStarted ? "ON" : "OFF",
               wroomStatus.barcode[0] ? wroomStatus.barcode : "---");
    }
    display.setTextColor(wroomStatus.wristPass ? TFT_GREEN : TFT_YELLOW, 0x0841);
  }
  display.drawString(esdText, 400, 305);

  char cameraText[90];
  if (!cameraOnline) {
    snprintf(cameraText, sizeof(cameraText), "Camera: OFFLINE");
    display.setTextColor(TFT_ORANGE, 0x0841);
  } else if (cameraStatus.enrollmentState == 1 || enrollmentRequestedFromHmi) {
    snprintf(cameraText, sizeof(cameraText), "REGISTERING: Look at camera");
    display.setTextColor(TFT_CYAN, 0x0841);
  } else if (cameraStatus.enrollmentState == 2) {
    snprintf(cameraText, sizeof(cameraText), "REGISTERED - Your ID: %02d",
             cameraStatus.faceId);
    display.setTextColor(TFT_GREEN, 0x0841);
  } else if (cameraStatus.enrollmentState == 3) {
    snprintf(cameraText, sizeof(cameraText), "REGISTRATION FAILED");
    display.setTextColor(TFT_RED, 0x0841);
  } else if (!cameraStatus.faceDetected) {
    snprintf(cameraText, sizeof(cameraText), "Camera: No face");
    display.setTextColor(TFT_YELLOW, 0x0841);
  } else if (cameraStatus.faceId >= 0) {
    snprintf(cameraText, sizeof(cameraText), "KNOWN USER - ID: %02d",
             cameraStatus.faceId);
    display.setTextColor(TFT_GREEN, 0x0841);
  } else {
    snprintf(cameraText, sizeof(cameraText), "UNKNOWN USER");
    display.setTextColor(TFT_RED, 0x0841);
  }
  display.drawString(cameraText, 400, 340);
}

void processEspNowMessages() {
  EsdNowMessage incoming{};
  bool hasMessage = false;
  portENTER_CRITICAL(&espNowMux);
  if (espNowPacketPending) {
    incoming = pendingEspNowMessage;
    espNowPacketPending = false;
    hasMessage = true;
  }
  portEXIT_CRITICAL(&espNowMux);
  if (!hasMessage) return;

  incoming.barcode[sizeof(incoming.barcode) - 1] = '\0';
  if (incoming.source == NODE_WROOM && incoming.type == MESSAGE_WROOM_STATUS) {
    wroomStatus = incoming;
    if (incoming.badgeReaderActive || incoming.barcode[0]) {
      badgeRequestPending = false;
    }
    lastWroomPacketMs = millis();
    Serial.printf("WROOM #%lu: low=%u high=%u pass=%u test=%u barcode=%s\n",
                  static_cast<unsigned long>(incoming.sequence), incoming.wristLow,
                  incoming.wristHigh, incoming.wristPass, incoming.testStarted,
                  incoming.barcode);
  } else if (incoming.source == NODE_CAMERA &&
             incoming.type == MESSAGE_CAMERA_STATUS) {
    cameraStatus = incoming;
    lastCameraPacketMs = millis();
    if (incoming.enrollmentState == 2) {
      lastEnrolledId = incoming.faceId;
      enrollmentRequestedFromHmi = false;
      playMelody();
    } else if (incoming.enrollmentState == 3) {
      enrollmentRequestedFromHmi = false;
    } else if (incoming.enrollmentState == 1) {
      enrollmentRequestedFromHmi = false;
    }
    Serial.printf("CAMERA #%lu: face=%u id=%d enrollment=%u\n",
                  static_cast<unsigned long>(incoming.sequence),
                  incoming.faceDetected, incoming.faceId,
                  incoming.enrollmentState);
  }
  drawRemoteStatus();
}

void playTone(uint16_t frequencyHz, uint16_t durationMs) {
  constexpr size_t FRAMES_PER_BUFFER = 128;
  int16_t stereoBuffer[FRAMES_PER_BUFFER * 2];
  const uint32_t totalFrames =
      (SPEAKER_SAMPLE_RATE * static_cast<uint32_t>(durationMs)) / 1000;
  const uint32_t halfPeriodFrames = SPEAKER_SAMPLE_RATE / (2 * frequencyHz);
  uint32_t framesWritten = 0;

  while (framesWritten < totalFrames) {
    const size_t framesThisPass =
        min(static_cast<uint32_t>(FRAMES_PER_BUFFER), totalFrames - framesWritten);

    for (size_t frame = 0; frame < framesThisPass; ++frame) {
      const uint32_t absoluteFrame = framesWritten + frame;
      const int16_t sample =
          ((absoluteFrame / halfPeriodFrames) % 2 == 0)
              ? SPEAKER_AMPLITUDE
              : -SPEAKER_AMPLITUDE;

      stereoBuffer[frame * 2] = sample;
      stereoBuffer[frame * 2 + 1] = sample;
    }

    speakerI2s.write(reinterpret_cast<uint8_t *>(stereoBuffer),
                     framesThisPass * 2 * sizeof(int16_t));
    framesWritten += framesThisPass;
  }
}

void playSilence(uint16_t durationMs) {
  constexpr size_t FRAMES_PER_BUFFER = 128;
  int16_t stereoBuffer[FRAMES_PER_BUFFER * 2] = {};
  uint32_t framesRemaining =
      (SPEAKER_SAMPLE_RATE * static_cast<uint32_t>(durationMs)) / 1000;

  while (framesRemaining > 0) {
    const size_t framesThisPass =
        min(static_cast<uint32_t>(FRAMES_PER_BUFFER), framesRemaining);
    speakerI2s.write(reinterpret_cast<uint8_t *>(stereoBuffer),
                     framesThisPass * 2 * sizeof(int16_t));
    framesRemaining -= framesThisPass;
  }
}

void playMelody() {
  if (!speakerReady) {
    Serial.println("Speaker is not available.");
    return;
  }

  playTone(523, 160);
  playSilence(50);
  playTone(659, 160);
  playSilence(50);
  playTone(784, 240);
  playSilence(80);
}

bool initializeSpeaker() {
  speakerI2s.setPins(SPEAKER_I2S_BCLK_PIN, SPEAKER_I2S_LRCLK_PIN,
                     SPEAKER_I2S_DATA_PIN);

  if (!speakerI2s.begin(I2S_MODE_STD, SPEAKER_SAMPLE_RATE,
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("Failed to initialize the I2S speaker.");
    return false;
  }

  Serial.println("I2S speaker initialized.");
  return true;
}

void drawPlayButton(bool pressed = false) {
  const uint32_t buttonColor = pressed ? 0x03EF : 0x057F;
  display.fillRoundRect(PLAY_BUTTON_X, PLAY_BUTTON_Y, PLAY_BUTTON_WIDTH,
                        PLAY_BUTTON_HEIGHT, 18, buttonColor);
  display.drawRoundRect(PLAY_BUTTON_X, PLAY_BUTTON_Y, PLAY_BUTTON_WIDTH,
                        PLAY_BUTTON_HEIGHT, 18, TFT_WHITE);
  display.setTextDatum(middle_center);
  display.setTextColor(TFT_WHITE, buttonColor);
  display.setTextSize(3);
  display.drawString("Click to play", PLAY_BUTTON_X + PLAY_BUTTON_WIDTH / 2,
                     PLAY_BUTTON_Y + PLAY_BUTTON_HEIGHT / 2);
}

void drawRegisterButton(bool pressed = false) {
  const uint32_t buttonColor = pressed ? 0x7A20 : 0xB320;
  display.fillRoundRect(REGISTER_BUTTON_X, REGISTER_BUTTON_Y,
                        REGISTER_BUTTON_WIDTH, REGISTER_BUTTON_HEIGHT, 18,
                        buttonColor);
  display.drawRoundRect(REGISTER_BUTTON_X, REGISTER_BUTTON_Y,
                        REGISTER_BUTTON_WIDTH, REGISTER_BUTTON_HEIGHT, 18,
                        TFT_WHITE);
  display.setTextDatum(middle_center);
  display.setTextColor(TFT_WHITE, buttonColor);
  display.setTextSize(3);
  display.drawString("Register face",
                     REGISTER_BUTTON_X + REGISTER_BUTTON_WIDTH / 2,
                     REGISTER_BUTTON_Y + REGISTER_BUTTON_HEIGHT / 2);
}

void drawBadgeButton(bool pressed = false) {
  const uint32_t buttonColor = pressed ? 0x03EF : 0x057F;
  display.fillRoundRect(BADGE_BUTTON_X, BADGE_BUTTON_Y, BADGE_BUTTON_WIDTH,
                        BADGE_BUTTON_HEIGHT, 18, buttonColor);
  display.drawRoundRect(BADGE_BUTTON_X, BADGE_BUTTON_Y, BADGE_BUTTON_WIDTH,
                        BADGE_BUTTON_HEIGHT, 18, TFT_WHITE);
  display.setTextDatum(middle_center);
  display.setTextColor(TFT_WHITE, buttonColor);
  display.setTextSize(3);
  display.drawString("Read Badge", BADGE_BUTTON_X + BADGE_BUTTON_WIDTH / 2,
                     BADGE_BUTTON_Y + BADGE_BUTTON_HEIGHT / 2);
}

void requestOneBadgeRead() {
  EsdNowMessage command{};
  command.magic = ESD_MESSAGE_MAGIC;
  command.version = ESD_PROTOCOL_VERSION;
  command.source = 0;
  command.type = MESSAGE_READ_BADGE;
  command.sequence = ++commandSequence;
  command.uptimeMs = millis();
  badgeRequestPending = true;
  for (uint8_t attempt = 0; attempt < 5; ++attempt) {
    esp_now_send(BROADCAST_ADDRESS,
                 reinterpret_cast<const uint8_t *>(&command), sizeof(command));
    delay(40);
  }
  Serial.println("One-shot badge read requested from HMI.");
  drawRemoteStatus();
}

void requestFaceEnrollment() {
  EsdNowMessage command{};
  command.magic = ESD_MESSAGE_MAGIC;
  command.version = ESD_PROTOCOL_VERSION;
  command.source = 0;
  command.type = MESSAGE_ENROLL_FACE;
  command.sequence = ++commandSequence;
  command.uptimeMs = millis();
  enrollmentRequestedFromHmi = true;
  // Retry the exact same transaction ID. The camera deduplicates it, so this
  // improves radio reliability without enrolling the same person repeatedly.
  for (uint8_t attempt = 0; attempt < 5; ++attempt) {
    esp_now_send(BROADCAST_ADDRESS,
                 reinterpret_cast<const uint8_t *>(&command), sizeof(command));
    delay(40);
  }
  Serial.println("Face enrollment requested from HMI.");
  drawRemoteStatus();
}

void drawEnvironmentValues() {
  display.fillRect(0, 360, 800, 80, 0x0841);
  display.setTextDatum(middle_center);
  display.setTextColor(TFT_WHITE, 0x0841);
  display.setTextSize(2);

  if (isnan(lastTemperatureC) || isnan(lastRelativeHumidity)) {
    display.drawString("Waiting for SHT31 measurements...", 400, 400);
    return;
  }

  char environmentText[80];
  snprintf(environmentText, sizeof(environmentText),
           "Temperature: %.2f C     Humidity: %.2f %%RH", lastTemperatureC,
           lastRelativeHumidity);
  display.drawString(environmentText, 400, 400);
}

void initializeInterface() {
  display.init();
  display.setRotation(0);
  display.setBrightness(220);
  display.fillScreen(0x0841);

  display.setTextDatum(middle_center);
  display.setTextColor(TFT_WHITE, 0x0841);
  display.setTextSize(3);
  display.drawString("ESD Environmental Monitor", 400, 80);

  display.setTextSize(2);
  display.setTextColor(0xBDF7, 0x0841);
  display.drawString("Press Register face, then look at the camera", 400, 125);

  drawRegisterButton();
  drawBadgeButton();
  drawRemoteStatus();
  drawEnvironmentValues();
}

bool pointIsInsidePlayButton(uint16_t x, uint16_t y) {
  return x >= PLAY_BUTTON_X && x < PLAY_BUTTON_X + PLAY_BUTTON_WIDTH &&
         y >= PLAY_BUTTON_Y && y < PLAY_BUTTON_Y + PLAY_BUTTON_HEIGHT;
}

bool pointIsInsideRegisterButton(uint16_t x, uint16_t y) {
  return x >= REGISTER_BUTTON_X &&
         x < REGISTER_BUTTON_X + REGISTER_BUTTON_WIDTH &&
         y >= REGISTER_BUTTON_Y &&
         y < REGISTER_BUTTON_Y + REGISTER_BUTTON_HEIGHT;
}

bool pointIsInsideBadgeButton(uint16_t x, uint16_t y) {
  return x >= BADGE_BUTTON_X && x < BADGE_BUTTON_X + BADGE_BUTTON_WIDTH &&
         y >= BADGE_BUTTON_Y && y < BADGE_BUTTON_Y + BADGE_BUTTON_HEIGHT;
}

bool readGt911Register(uint16_t reg, uint8_t *data, size_t length) {
  Wire.beginTransmission(activeGt911Address);
  Wire.write(static_cast<uint8_t>(reg >> 8));
  Wire.write(static_cast<uint8_t>(reg & 0xFF));

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(activeGt911Address, length);
  if (received != length) {
    return false;
  }

  for (size_t index = 0; index < length; ++index) {
    data[index] = Wire.read();
  }

  return true;
}

void clearGt911Status() {
  Wire.beginTransmission(activeGt911Address);
  Wire.write(static_cast<uint8_t>(GT911_STATUS_REGISTER >> 8));
  Wire.write(static_cast<uint8_t>(GT911_STATUS_REGISTER & 0xFF));
  Wire.write(0);
  Wire.endTransmission();
}

bool selectGt911Address() {
  const uint8_t candidates[] = {GT911_ADDRESS_PRIMARY,
                                GT911_ADDRESS_SECONDARY};
  for (uint8_t address : candidates) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      activeGt911Address = address;
      Serial.printf("GT911 touch controller detected at 0x%02X.\n",
                    activeGt911Address);
      return true;
    }
  }
  Serial.println("GT911 touch controller not detected at 0x14 or 0x5D.");
  return false;
}

bool readTouch(uint16_t &x, uint16_t &y) {
  uint8_t status = 0;
  if (!readGt911Register(GT911_STATUS_REGISTER, &status, 1)) {
    return false;
  }

  const bool dataReady = (status & 0x80) != 0;
  const uint8_t touchCount = status & 0x0F;

  if (!dataReady) {
    return false;
  }

  if (touchCount == 0 || touchCount > 5) {
    clearGt911Status();
    return false;
  }

  uint8_t pointData[8] = {};
  if (!readGt911Register(GT911_FIRST_POINT_REGISTER, pointData,
                         sizeof(pointData))) {
    clearGt911Status();
    return false;
  }

  const uint16_t rawX = pointData[1] | (pointData[2] << 8);
  const uint16_t rawY = pointData[3] | (pointData[4] << 8);
  clearGt911Status();

  // On this Elecrow panel the GT911 X axis already matches the displayed X
  // axis. Only Y is mounted opposite to the display's native orientation.
  x = constrain(rawX, 0, 799);
  y = constrain(480 - rawY, 0, 479);
  return true;
}

void printEnvironmentValues() {
  if (isnan(lastTemperatureC) || isnan(lastRelativeHumidity)) {
    Serial.println("Environment: SHT31 measurement not available.");
    return;
  }

  Serial.print("Environment on button press -> Temperature: ");
  Serial.print(lastTemperatureC, 2);
  Serial.print(" C | Humidity: ");
  Serial.print(lastRelativeHumidity, 2);
  Serial.println(" %RH");
}

void handleTouch() {
  uint16_t touchX = 0;
  uint16_t touchY = 0;
  const bool touched = readTouch(touchX, touchY);

  if (touched && !touchWasDown &&
      pointIsInsideRegisterButton(touchX, touchY)) {
    Serial.printf("Register touch: x=%u, y=%u\n", touchX, touchY);
    drawRegisterButton(true);
    requestFaceEnrollment();
  } else if (touched && !touchWasDown &&
             pointIsInsideBadgeButton(touchX, touchY)) {
    Serial.printf("Badge touch: x=%u, y=%u\n", touchX, touchY);
    drawBadgeButton(true);
    requestOneBadgeRead();
  }

  if (!touched && touchWasDown) {
    drawRegisterButton(false);
    drawBadgeButton(false);
  }

  touchWasDown = touched;
}

uint8_t scanI2cBus() {
  uint8_t deviceCount = 0;

  Serial.println("Scanning I2C bus...");

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 0x10) {
        Serial.print('0');
      }
      Serial.println(address, HEX);
      ++deviceCount;
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found.");
  }

  return deviceCount;
}

bool initializeSht31() {
  if (sht31.begin(SHT31_ADDRESS_PRIMARY)) {
    activeSht31Address = SHT31_ADDRESS_PRIMARY;
    return true;
  }

  if (sht31.begin(SHT31_ADDRESS_SECONDARY)) {
    activeSht31Address = SHT31_ADDRESS_SECONDARY;
    return true;
  }

  activeSht31Address = 0;
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESD System - Principal Microcontroller");
  Serial.println("SHT31 temperature and humidity monitor");
  Serial.println("========================================");

  initializeInterface();
  speakerReady = initializeSpeaker();
  initializeEspNow();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(100);

  scanI2cBus();
  selectGt911Address();
  sht31Detected = initializeSht31();

  if (sht31Detected) {
    Serial.print("SHT31 detected at I2C address 0x");
    Serial.println(activeSht31Address, HEX);
  } else {
    Serial.println("SHT31 not detected at 0x44 or 0x45.");
    Serial.println("Check power, ground, SDA GPIO19, and SCL GPIO20.");
  }
}

void loop() {
  const unsigned long now = millis();

  handleTouch();
  processEspNowMessages();

  static unsigned long lastRemoteRefreshMs = 0;
  if (now - lastRemoteRefreshMs >= 1000) {
    lastRemoteRefreshMs = now;
    drawRemoteStatus();
  }

  if (!sht31Detected) {
    if (now - lastRetryTime >= SENSOR_RETRY_INTERVAL_MS) {
      lastRetryTime = now;
      Serial.println("Retrying SHT31 detection...");
      scanI2cBus();
      sht31Detected = initializeSht31();

      if (sht31Detected) {
        Serial.print("SHT31 detected at I2C address 0x");
        Serial.println(activeSht31Address, HEX);
      }
    }
  } else if (now - lastReadTime >= READ_INTERVAL_MS) {
    lastReadTime = now;

    lastTemperatureC = sht31.readTemperature();
    lastRelativeHumidity = sht31.readHumidity();

    if (isnan(lastTemperatureC) || isnan(lastRelativeHumidity)) {
      Serial.println("SHT31 read failed; sensor will be reinitialized.");
      sht31Detected = false;
      lastRetryTime = now;
      lastTemperatureC = NAN;
      lastRelativeHumidity = NAN;
    } else {
      Serial.print("Temperature: ");
      Serial.print(lastTemperatureC, 2);
      Serial.print(" C | Humidity: ");
      Serial.print(lastRelativeHumidity, 2);
      Serial.println(" %RH");
    }

    drawEnvironmentValues();
  }

  delay(10);
}
