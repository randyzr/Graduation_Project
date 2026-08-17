#pragma once

#include <Arduino.h>

// Every controller includes an identical copy of this communication contract.
// Changing the field order or type changes the bytes sent over the radio, so all
// three copies must always stay identical.

constexpr uint8_t ESD_ESPNOW_CHANNEL = 6;
constexpr uint32_t ESD_MESSAGE_MAGIC = 0x45534431;  // "ESD1"
constexpr uint8_t ESD_PROTOCOL_VERSION = 1;

enum EsdNode : uint8_t {
  NODE_WROOM = 1,
  NODE_CAMERA = 2,
};

enum EsdMessageType : uint8_t {
  MESSAGE_WROOM_STATUS = 1,
  MESSAGE_CAMERA_STATUS = 2,
  MESSAGE_ENROLL_FACE = 3,
  MESSAGE_READ_BADGE = 4,
  MESSAGE_DELETE_ALL_FACES = 5,
};

struct __attribute__((packed)) EsdNowMessage {
  // Header: validates the packet and identifies its purpose.
  uint32_t magic;
  uint8_t version;
  uint8_t source;
  uint8_t type;
  uint8_t reserved;
  uint32_t sequence;
  uint32_t uptimeMs;
  // HZR-171 signals supplied by SecondaryController.
  uint8_t wristLow;
  uint8_t wristHigh;
  uint8_t wristPass;
  uint8_t testStarted;
  // Face result supplied by FacialController.
  uint8_t faceDetected;
  int16_t faceId;          // -2=no face, -1=unknown, 0+=known ID
  uint8_t enrollmentState; // 0=idle, 1=waiting, 2=success, 3=failed/full
  uint8_t badgeReaderActive;
  // One completed GM67 badge value; empty when no new badge is available.
  char barcode[48];
};

static_assert(sizeof(EsdNowMessage) <= 250,
              "ESP-NOW packet must remain within the payload limit");
