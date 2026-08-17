#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

constexpr int SD_MOSI_PIN = 11;
constexpr int SD_MISO_PIN = 13;
constexpr int SD_SCK_PIN = 12;
constexpr int SD_CS_PIN = 10;
constexpr char TEST_PATH[] = "/codex_hmi_sd_test.txt";
constexpr char TEST_CONTENT[] = "Elecrow HMI SD write/read test";

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("HMI_SD_TEST_BEGIN");

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, SPI, 1000000)) {
    Serial.println("HMI_SD_TEST_FAIL: mount");
    return;
  }
  if (SD.cardType() == CARD_NONE) {
    Serial.println("HMI_SD_TEST_FAIL: no card detected");
    return;
  }

  const char *type = "UNKNOWN";
  if (SD.cardType() == CARD_MMC) type = "MMC";
  if (SD.cardType() == CARD_SD) type = "SDSC";
  if (SD.cardType() == CARD_SDHC) type = "SDHC/SDXC";
  Serial.printf("HMI_SD_CARD_TYPE: %s\n", type);
  Serial.printf("HMI_SD_CARD_SIZE_MB: %llu\n", SD.cardSize() / (1024ULL * 1024ULL));
  Serial.printf("HMI_SD_TOTAL_MB: %llu\n", SD.totalBytes() / (1024ULL * 1024ULL));
  Serial.printf("HMI_SD_USED_MB: %llu\n", SD.usedBytes() / (1024ULL * 1024ULL));

  File output = SD.open(TEST_PATH, FILE_WRITE);
  if (!output || output.print(TEST_CONTENT) != strlen(TEST_CONTENT)) {
    Serial.println("HMI_SD_TEST_FAIL: write");
    if (output) output.close();
    return;
  }
  output.close();

  File input = SD.open(TEST_PATH, FILE_READ);
  String content = input ? input.readString() : "";
  if (input) input.close();
  if (content != TEST_CONTENT) {
    Serial.printf("HMI_SD_TEST_FAIL: readback [%s]\n", content.c_str());
    return;
  }
  if (!SD.remove(TEST_PATH)) {
    Serial.println("HMI_SD_TEST_FAIL: cleanup");
    return;
  }

  Serial.println("HMI_SD_TEST_PASS: mount, write, read, and cleanup succeeded");
}

void loop() {
  delay(1000);
}
