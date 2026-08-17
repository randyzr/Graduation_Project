#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>

constexpr int SD_CLK_PIN = 39;
constexpr int SD_CMD_PIN = 38;
constexpr int SD_D0_PIN = 40;
constexpr char TEST_PATH[] = "/codex_sd_test.txt";
constexpr char TEST_CONTENT[] = "Freenove ESP32-S3 SD write/read test";

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("SD_TEST_BEGIN");

  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
  if (!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 5)) {
    Serial.println("SD_TEST_FAIL: mount");
    return;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("SD_TEST_FAIL: no card detected");
    return;
  }

  const char *type = "UNKNOWN";
  if (SD_MMC.cardType() == CARD_MMC) type = "MMC";
  if (SD_MMC.cardType() == CARD_SD) type = "SDSC";
  if (SD_MMC.cardType() == CARD_SDHC) type = "SDHC/SDXC";
  Serial.printf("SD_CARD_TYPE: %s\n", type);
  Serial.printf("SD_CARD_SIZE_MB: %llu\n", SD_MMC.cardSize() / (1024ULL * 1024ULL));
  Serial.printf("SD_TOTAL_MB: %llu\n", SD_MMC.totalBytes() / (1024ULL * 1024ULL));
  Serial.printf("SD_USED_MB: %llu\n", SD_MMC.usedBytes() / (1024ULL * 1024ULL));

  File output = SD_MMC.open(TEST_PATH, FILE_WRITE);
  if (!output || output.print(TEST_CONTENT) != strlen(TEST_CONTENT)) {
    Serial.println("SD_TEST_FAIL: write");
    if (output) output.close();
    return;
  }
  output.close();

  File input = SD_MMC.open(TEST_PATH, FILE_READ);
  String content = input ? input.readString() : "";
  if (input) input.close();
  if (content != TEST_CONTENT) {
    Serial.printf("SD_TEST_FAIL: readback [%s]\n", content.c_str());
    return;
  }

  if (!SD_MMC.remove(TEST_PATH)) {
    Serial.println("SD_TEST_FAIL: cleanup");
    return;
  }

  Serial.println("SD_TEST_PASS: mount, write, read, and cleanup succeeded");
}

void loop() {
  delay(1000);
}
