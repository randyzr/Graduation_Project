#include "esp_camera.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "EspNowProtocol.h"

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

const char *cameraAccessPoint = "ESD-CAMERA";
const char *cameraAccessPointPassword = "ESDsystem2026";
const uint8_t BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

EsdNowMessage cameraMessage{};
uint32_t cameraSequence = 0;
uint32_t lastCameraSendMs = 0;
bool lastReportedFaceState = false;
volatile bool enrollmentRequested = false;
volatile uint32_t lastEnrollmentCommand = 0;
int16_t currentFaceId = -2;
uint8_t currentEnrollmentState = 0;

void startCameraServer();
void setupLedFlash(int pin);
void startAutonomousFaceRecognition();

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data,
                     int length) {
  if (length != sizeof(EsdNowMessage)) return;
  EsdNowMessage message{};
  memcpy(&message, data, sizeof(message));
  if (message.magic == ESD_MESSAGE_MAGIC &&
      message.version == ESD_PROTOCOL_VERSION &&
      message.type == MESSAGE_ENROLL_FACE &&
      message.sequence != lastEnrollmentCommand) {
    lastEnrollmentCommand = message.sequence;
    enrollmentRequested = true;
  }
}

bool consumeEnrollmentRequest() {
  if (!enrollmentRequested) return false;
  enrollmentRequested = false;
  return true;
}

bool initializeEspNow() {
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
  Serial.printf("ESP-NOW camera sender ready on channel %u.\n",
                ESD_ESPNOW_CHANNEL);
  return true;
}

void sendCameraStatus(bool faceDetected) {
  cameraMessage.magic = ESD_MESSAGE_MAGIC;
  cameraMessage.version = ESD_PROTOCOL_VERSION;
  cameraMessage.source = NODE_CAMERA;
  cameraMessage.type = MESSAGE_CAMERA_STATUS;
  cameraMessage.sequence = ++cameraSequence;
  cameraMessage.uptimeMs = millis();
  cameraMessage.faceDetected = faceDetected;
  cameraMessage.faceId = currentFaceId;
  cameraMessage.enrollmentState = enrollmentRequested ? 1 : currentEnrollmentState;
  esp_now_send(BROADCAST_ADDRESS,
               reinterpret_cast<const uint8_t *>(&cameraMessage),
               sizeof(cameraMessage));
  lastCameraSendMs = millis();
  lastReportedFaceState = faceDetected;
}

void notifyFaceRecognition(bool faceDetected, int16_t faceId,
                           uint8_t enrollmentState) {
  currentFaceId = faceId;
  currentEnrollmentState = enrollmentState;
  cameraMessage.magic = ESD_MESSAGE_MAGIC;
  cameraMessage.version = ESD_PROTOCOL_VERSION;
  cameraMessage.source = NODE_CAMERA;
  cameraMessage.type = MESSAGE_CAMERA_STATUS;
  cameraMessage.sequence = ++cameraSequence;
  cameraMessage.uptimeMs = millis();
  cameraMessage.faceDetected = faceDetected;
  cameraMessage.faceId = faceId;
  cameraMessage.enrollmentState = enrollmentState;
  esp_now_send(BROADCAST_ADDRESS,
               reinterpret_cast<const uint8_t *>(&cameraMessage),
               sizeof(cameraMessage));
  lastCameraSendMs = millis();
  lastReportedFaceState = faceDetected;
}

// Called by app_httpd.cpp after processing each streamed camera frame.
void notifyFaceDetection(bool faceDetected) {
  const uint32_t now = millis();
  if (faceDetected != lastReportedFaceState || now - lastCameraSendMs >= 1000) {
    sendCameraStatus(faceDetected);
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("=================================");
  Serial.println("Freenove ESP32-S3 CameraWebServer");
  Serial.println("=================================");

  Serial.print("PSRAM detected: ");
  Serial.println(psramFound() ? "YES" : "NO");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // Safer clock for first recognition test
  config.xclk_freq_hz = 10000000;

  // For web streaming
  config.pixel_format = PIXFORMAT_JPEG;

  // Important: do not start with UXGA.
  // QVGA is better for face detection/recognition stability.
  config.frame_size = FRAMESIZE_QVGA;     // 320 x 240
  config.jpeg_quality = 12;
  config.fb_count = 1;

  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  Serial.println("Before camera init...");
  esp_err_t err = esp_camera_init(&config);
  Serial.println("After camera init...");

  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  Serial.println("Camera initialized successfully.");

  sensor_t *s = esp_camera_sensor_get();

  if (s == NULL) {
    Serial.println("Camera sensor pointer is NULL.");
    return;
  }

  // Stable settings for first test
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_quality(s, 12);

  // The camera is physically mounted upside down relative to the required
  // portrait view. These two sensor transforms add a 180-degree correction;
  // app_httpd.cpp then performs the required 90-degree portrait rotation.
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  Serial.println("Camera sensor settings applied.");

#if defined(LED_GPIO_NUM)
  if (LED_GPIO_NUM >= 0) {
    setupLedFlash(LED_GPIO_NUM);
  }
#endif

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  if (esp_wifi_set_channel(ESD_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Failed to set the ESP-NOW channel.");
    return;
  }
  initializeEspNow();
  sendCameraStatus(false);

  startAutonomousFaceRecognition();

  Serial.println("Camera ready in autonomous recognition mode (no stream/AP).");
}

void loop() {
  if (millis() - lastCameraSendMs >= 1000) {
    sendCameraStatus(lastReportedFaceState);
  }
  delay(20);
}
