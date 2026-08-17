#pragma once

#include <Arduino.h>

constexpr uint8_t ESD_ESPNOW_CHANNEL = 6;
constexpr uint32_t ESD_MESSAGE_MAGIC = 0x45534431;
constexpr uint8_t ESD_PROTOCOL_VERSION = 1;

enum EsdNode : uint8_t { NODE_WROOM = 1, NODE_CAMERA = 2 };
enum EsdMessageType : uint8_t {
  MESSAGE_WROOM_STATUS = 1,
  MESSAGE_CAMERA_STATUS = 2,
  MESSAGE_ENROLL_FACE = 3,
  MESSAGE_READ_BADGE = 4,
};

struct __attribute__((packed)) EsdNowMessage {
  uint32_t magic;
  uint8_t version;
  uint8_t source;
  uint8_t type;
  uint8_t reserved;
  uint32_t sequence;
  uint32_t uptimeMs;
  uint8_t wristLow;
  uint8_t wristHigh;
  uint8_t wristPass;
  uint8_t testStarted;
  uint8_t faceDetected;
  int16_t faceId;
  uint8_t enrollmentState;
  uint8_t badgeReaderActive;
  char barcode[48];
};
