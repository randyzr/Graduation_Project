#pragma once

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Preferences.h>
#include <SD.h>

// =============================================================================
// HMI WORKFLOW: SCREEN STATE MACHINE AND USER DATA
// =============================================================================
// Think of this class as the station's rule book. "screen_" is the current
// state. Events such as a touch, camera packet, badge, or tester packet update
// stored data and may call changeScreen() to move to the next state.
//
// Learning map:
//   - tick(): handles time-based transitions.
//   - onFaceStatus(), onBadge(), onTestStatus(): receive external events.
//   - handleTouch(): translates screen touches into actions.
//   - changeScreen() and draw(): perform state transitions and rendering.
//   - initializeReport()/appendCurrentTestRecord(): manage the CSV report.

class HmiWorkflowDraft {
 public:
  static constexpr uint32_t TEST_SAMPLE_DELAY_MS = 2000;
  static constexpr uint32_t FACE_RESULT_DISPLAY_MS = 600;
  static constexpr uint32_t FACE_CACHE_MAX_AGE_MS = 1500;

  enum class Screen : uint8_t {
    Welcome,
    FaceScan,
    BadgeScan,
    Registration,
    FaceConsent,
    FaceEnrollment,
    Test,
    Failure,
    Thanks,
    Report,
    StoredUsers,
    Diagnostic,
  };

  enum class Sound : uint8_t { None, Success, Failure };

  explicit HmiWorkflowDraft(Elecrow7InchHMI &display, bool sdCardReady)
      : display_(display), sdCardReady_(sdCardReady) {}

  void begin() {
    initializeRegistry();
    initializeReport();
    changeScreen(Screen::Welcome);
  }

  void setEnvironment(float temperatureC, float humidity) {
    temperatureC_ = temperatureC;
    humidity_ = humidity;
    drawFooter();
  }

  void tick(uint32_t now) {
    // Timers are checked without delay(), allowing communication and touch to
    // continue while a screen is waiting.
    if (screen_ == Screen::FaceScan && recognizedFaceId_ >= 0 &&
        timerExpired(now, screenStartedMs_, FACE_RESULT_DISPLAY_MS)) {
      completeFacialIdentification();
    } else if (screen_ == Screen::FaceScan &&
               timerExpired(now, screenStartedMs_, 5000)) {
      changeScreen(Screen::BadgeScan);
      badgeRequest_ = true;
    }

    if (screen_ == Screen::BadgeScan && timerExpired(now, screenStartedMs_, 10000)) {
      changeScreen(Screen::Welcome);
    }

    if (screen_ == Screen::Test && testHolding_) {
      const uint32_t heldMs = timerExpired(now, testHoldStartedMs_, 0)
                                  ? now - testHoldStartedMs_
                                  : 0;
      drawTestProgress(heldMs);
      if (heldMs >= TEST_SAMPLE_DELAY_MS && !testEvaluated_) {
        // Take one immutable snapshot exactly when the two-second window ends.
        // Later ESP-NOW packets cannot change the result being recorded.
        finalLow_ = latestLow_;
        finalHigh_ = latestHigh_;
        finalPass_ = latestPass_;
        testEvaluated_ = true;
        Serial.printf("ESD final sample: LOW=%u HIGH=%u OK=%u\n", finalLow_,
                      finalHigh_, finalPass_);
        appendCurrentTestRecord();
        if (finalPass_) {
          pendingSound_ = Sound::Success;
          changeScreen(Screen::Thanks);
        } else {
          pendingSound_ = Sound::Failure;
          changeScreen(Screen::Failure);
        }
      }
    }

    if (screen_ == Screen::Thanks && timerExpired(now, screenStartedMs_, 5000)) {
      changeScreen(Screen::Welcome);
    }

    if (screen_ == Screen::Report && reportDeleteConfirmation_ &&
        timerExpired(now, reportDeleteConfirmationMs_, 5000)) {
      reportDeleteConfirmation_ = false;
      drawReport();
    }

    if (screen_ == Screen::StoredUsers && userDeleteConfirmation_ &&
        timerExpired(now, userDeleteConfirmationMs_, 5000)) {
      userDeleteConfirmation_ = false;
      drawStoredUsers();
    }
  }

  void onFaceStatus(bool faceDetected, int16_t faceId) {
    const uint32_t now = millis();
    if (faceDetected && faceId >= 0) {
      cachedFaceId_ = faceId;
      cachedFaceMs_ = now;
      if (screen_ == Screen::FaceScan) recognizedFaceId_ = faceId;
    } else {
      // Never reuse an identity after the recognized face leaves the camera.
      cachedFaceId_ = -1;
      cachedFaceMs_ = 0;
    }
  }

  void onBadge(const char *badge) {
    if (screen_ != Screen::BadgeScan || badge == nullptr || badge[0] == '\0') {
      return;
    }

    currentBadge_ = badge;
    const int8_t userIndex = findUserByBadge(currentBadge_.c_str());
    if (userIndex >= 0) {
      currentUser_ = userFullName(users_[userIndex]);
      changeScreen(Screen::Test);
    } else {
      firstName_ = "";
      lastName_ = "";
      activeField_ = 0;
      changeScreen(Screen::Registration);
    }
  }

  void onEnrollmentStatus(int16_t faceId, uint8_t enrollmentState) {
    if (screen_ != Screen::FaceEnrollment) return;
    // Ignore any enrollment state left in a heartbeat from the previous user
    // while the new ESP-NOW command is reaching the camera.
    if (!timerExpired(millis(), screenStartedMs_, 800)) return;
    if (enrollmentState == 2 && faceId >= 0) {
      saveCurrentUser(faceId);
      currentUser_ = firstName_ + " " + lastName_;
      pendingSound_ = Sound::Success;
      changeScreen(Screen::Test);
    } else if (enrollmentState == 3) {
      enrollmentFailed_ = true;
      drawFaceEnrollment();
    }
  }

  void onTestStatus(bool testStarted, bool low, bool high, bool pass,
                    uint32_t now) {
    const bool diagnosticChanged = latestLow_ != low || latestHigh_ != high ||
                                   latestPass_ != pass ||
                                   latestButtonPressed_ != testStarted;
    latestLow_ = low;
    latestHigh_ = high;
    latestPass_ = pass;
    latestButtonPressed_ = testStarted;
    if (screen_ == Screen::Diagnostic) {
      if (diagnosticChanged) drawDiagnosticSignals();
      return;
    }
    if (screen_ != Screen::Test) return;

    // BP starts one fixed window. A later BP=LOW does not cancel that window.
    if (testStarted && !testHolding_) {
      testHolding_ = true;
      testEvaluated_ = false;
      testHoldStartedMs_ = now;
      Serial.println("ESD two-second evaluation window started.");
    }
  }

  void handleTouch(uint16_t x, uint16_t y) {
    if (static_cast<int32_t>(millis() - touchesEnabledAfterMs_) < 0) return;

    switch (screen_) {
      case Screen::Welcome:
        if (inside(x, y, 285, 340, 230, 45)) {
          changeScreen(Screen::FaceScan);
          if (cachedFaceId_ >= 0 &&
              !timerExpired(millis(), cachedFaceMs_, FACE_CACHE_MAX_AGE_MS)) {
            recognizedFaceId_ = cachedFaceId_;
            Serial.printf("Using warm camera recognition: face ID %d.\n",
                          recognizedFaceId_);
          }
        } else if (inside(x, y, 285, 400, 230, 45)) {
          reportOffset_ = 0;
          refreshReportCount();
          changeScreen(Screen::Report);
        } else if (inside(x, y, 25, 400, 230, 45)) {
          userDeleteConfirmation_ = false;
          changeScreen(Screen::StoredUsers);
        } else if (inside(x, y, 545, 400, 230, 45)) {
          changeScreen(Screen::Diagnostic);
        }
        break;

      case Screen::FaceScan:
        if (inside(x, y, 275, 365, 250, 55)) {
          currentUser_ = "Draft User";
          currentBadge_ = "00000000";
          changeScreen(Screen::Test);
        }
        break;

      case Screen::BadgeScan:
        if (inside(x, y, 275, 365, 250, 55)) {
          char simulatedBadge[9];
          for (uint8_t index = 0; index < 8; ++index) {
            simulatedBadge[index] = static_cast<char>('0' + random(10));
          }
          simulatedBadge[8] = '\0';
          Serial.printf("Simulated badge ID: %s\n", simulatedBadge);
          onBadge(simulatedBadge);
        }
        break;

      case Screen::Registration:
        handleRegistrationTouch(x, y);
        break;

      case Screen::FaceConsent:
        if (inside(x, y, 150, 245, 220, 90)) {
          enrollmentFailed_ = false;
          enrollmentRequest_ = true;
          changeScreen(Screen::FaceEnrollment);
        } else if (inside(x, y, 430, 245, 220, 90)) {
          saveCurrentUser(-1);
          currentUser_ = firstName_ + " " + lastName_;
          changeScreen(Screen::Test);
        }
        break;

      case Screen::FaceEnrollment:
        if (enrollmentFailed_ && inside(x, y, 150, 360, 220, 60)) {
          enrollmentFailed_ = false;
          enrollmentRequest_ = true;
          drawFaceEnrollment();
        } else if (enrollmentFailed_ && inside(x, y, 430, 360, 220, 60)) {
          saveCurrentUser(-1);
          currentUser_ = firstName_ + " " + lastName_;
          changeScreen(Screen::Test);
        }
        break;

      case Screen::Test:
        if (inside(x, y, 185, 365, 200, 55)) {
          finalLow_ = false;
          finalHigh_ = false;
          finalPass_ = true;
          appendCurrentTestRecord();
          pendingSound_ = Sound::Success;
          changeScreen(Screen::Thanks);
        } else if (inside(x, y, 415, 365, 200, 55)) {
          finalLow_ = false;
          finalHigh_ = false;
          finalPass_ = false;
          appendCurrentTestRecord();
          pendingSound_ = Sound::Failure;
          changeScreen(Screen::Failure);
        }
        break;

      case Screen::Failure:
        if (inside(x, y, 265, 375, 270, 60)) changeScreen(Screen::Test);
        break;

      case Screen::Thanks:
        if (inside(x, y, 275, 365, 250, 55)) changeScreen(Screen::Welcome);
        break;

      case Screen::Report:
        if (inside(x, y, 35, 390, 170, 45)) {
          reportDeleteConfirmation_ = false;
          changeScreen(Screen::Welcome);
        } else if (inside(x, y, 250, 390, 130, 45) && reportOffset_ > 0) {
          reportDeleteConfirmation_ = false;
          --reportOffset_;
          drawReport();
        } else if (inside(x, y, 420, 390, 130, 45) &&
                   reportOffset_ + REPORT_ROWS < reportRecordCount_) {
          reportDeleteConfirmation_ = false;
          ++reportOffset_;
          drawReport();
        } else if (inside(x, y, 590, 390, 180, 45) &&
                   reportRecordCount_ > 0) {
          if (reportDeleteConfirmation_) {
            deleteAllReportRecords();
          } else {
            reportDeleteConfirmation_ = true;
            reportDeleteConfirmationMs_ = millis();
            drawReport();
          }
        }
        break;

      case Screen::StoredUsers:
        if (inside(x, y, 110, 390, 220, 45)) {
          userDeleteConfirmation_ = false;
          changeScreen(Screen::Welcome);
        } else if (inside(x, y, 470, 390, 220, 45) && userCount() > 0) {
          if (userDeleteConfirmation_) {
            deleteAllUsers();
          } else {
            userDeleteConfirmation_ = true;
            userDeleteConfirmationMs_ = millis();
            drawStoredUsers();
          }
        }
        break;

      case Screen::Diagnostic:
        if (inside(x, y, 300, 375, 200, 55)) {
          changeScreen(Screen::Welcome);
        }
        break;
    }
  }

  bool consumeBadgeRequest() {
    const bool result = badgeRequest_;
    badgeRequest_ = false;
    return result;
  }

  bool consumeEnrollmentRequest() {
    const bool result = enrollmentRequest_;
    enrollmentRequest_ = false;
    return result;
  }

  bool consumeDeleteFacesRequest() {
    const bool result = deleteFacesRequest_;
    deleteFacesRequest_ = false;
    return result;
  }

  Sound consumeSound() {
    const Sound result = pendingSound_;
    pendingSound_ = Sound::None;
    return result;
  }

  Screen screen() const { return screen_; }

 private:
  static constexpr uint8_t MAX_USERS = 7;
  static constexpr uint8_t REPORT_ROWS = 7;
  static constexpr char REPORT_PATH[] = "/esd_report.csv";
  struct UserRecord {
    uint8_t used;
    int16_t faceId;
    char badge[48];
    char firstName[21];
    char lastName[21];
  };

  static constexpr uint16_t BACKGROUND = 0x0841;
  static constexpr uint16_t CARD = 0x10A2;
  static constexpr uint16_t BLUE = 0x057F;
  static constexpr uint16_t GREEN = 0x2E66;
  static constexpr uint16_t RED = 0xC986;
  static constexpr uint16_t MUTED = 0xBDF7;

  Elecrow7InchHMI &display_;
  // ----- Current state and data owned by the workflow ------------------------
  Screen screen_ = Screen::Welcome;
  uint32_t screenStartedMs_ = 0;
  uint32_t touchesEnabledAfterMs_ = 0;
  float temperatureC_ = NAN;
  float humidity_ = NAN;
  String currentBadge_;
  String currentUser_;
  String firstName_;
  String lastName_;
  UserRecord users_[MAX_USERS]{};
  int16_t recognizedFaceId_ = -1;
  int16_t cachedFaceId_ = -1;
  uint32_t cachedFaceMs_ = 0;
  uint8_t activeField_ = 0;
  bool badgeRequest_ = false;
  bool enrollmentRequest_ = false;
  bool enrollmentFailed_ = false;
  bool deleteFacesRequest_ = false;
  Sound pendingSound_ = Sound::None;
  bool testHolding_ = false;
  bool testEvaluated_ = false;
  uint32_t testHoldStartedMs_ = 0;
  bool latestLow_ = false;
  bool latestHigh_ = false;
  bool latestPass_ = false;
  bool latestButtonPressed_ = false;
  bool finalLow_ = false;
  bool finalHigh_ = false;
  bool finalPass_ = false;
  bool sdCardReady_ = false;
  uint16_t reportRecordCount_ = 0;
  uint16_t reportOffset_ = 0;
  bool reportDeleteConfirmation_ = false;
  uint32_t reportDeleteConfirmationMs_ = 0;
  bool userDeleteConfirmation_ = false;
  uint32_t userDeleteConfirmationMs_ = 0;

  bool inside(uint16_t x, uint16_t y, int16_t left, int16_t top,
              int16_t width, int16_t height) const {
    return x >= left && x < left + width && y >= top && y < top + height;
  }

  bool timerExpired(uint32_t now, uint32_t started,
                    uint32_t duration) const {
    return static_cast<int32_t>(now - started) >=
           static_cast<int32_t>(duration);
  }

  void completeFacialIdentification() {
    const int8_t userIndex = findUserByFace(recognizedFaceId_);
    if (userIndex >= 0) {
      currentUser_ = userFullName(users_[userIndex]);
      currentBadge_ = users_[userIndex].badge;
    } else {
      currentUser_ = "Recognized user";
      currentBadge_ = "00000000";
    }
    Serial.printf("Facial identification accepted: face ID %d.\n",
                  recognizedFaceId_);
    changeScreen(Screen::Test);
  }

  void initializeRegistry() {
    Preferences preferences;
    preferences.begin("esd-users", false);
    const bool resetRequired = !preferences.getBool("clean-v4", false);
    if (resetRequired) {
      memset(users_, 0, sizeof(users_));
      preferences.putBytes("records", users_, sizeof(users_));
      preferences.putBool("clean-v4", true);
      Serial.println("HMI user registry cleared for face database reset v4.");
    } else if (preferences.getBytesLength("records") == sizeof(users_)) {
      preferences.getBytes("records", users_, sizeof(users_));
    }
    preferences.end();
  }

  void persistRegistry() {
    Preferences preferences;
    preferences.begin("esd-users", false);
    preferences.putBytes("records", users_, sizeof(users_));
    preferences.end();
  }

  int8_t findUserByBadge(const char *badge) const {
    for (uint8_t index = 0; index < MAX_USERS; ++index) {
      if (users_[index].used && strcmp(users_[index].badge, badge) == 0) {
        return index;
      }
    }
    return -1;
  }

  int8_t findUserByFace(int16_t faceId) const {
    for (uint8_t index = 0; index < MAX_USERS; ++index) {
      if (users_[index].used && users_[index].faceId == faceId) return index;
    }
    return -1;
  }

  String userFullName(const UserRecord &user) const {
    return String(user.firstName) + " " + String(user.lastName);
  }

  uint8_t userCount() const {
    uint8_t count = 0;
    for (uint8_t index = 0; index < MAX_USERS; ++index) {
      if (users_[index].used) ++count;
    }
    return count;
  }

  void deleteAllUsers() {
    memset(users_, 0, sizeof(users_));
    persistRegistry();
    currentBadge_ = "";
    currentUser_ = "";
    cachedFaceId_ = -1;
    cachedFaceMs_ = 0;
    recognizedFaceId_ = -1;
    userDeleteConfirmation_ = false;
    deleteFacesRequest_ = true;
    Serial.println("All HMI users deleted; camera face-database deletion requested.");
    drawStoredUsers();
  }

  void initializeReport() {
    if (!sdCardReady_) return;
    if (!SD.exists(REPORT_PATH)) {
      File file = SD.open(REPORT_PATH, FILE_WRITE);
      if (!file) {
        sdCardReady_ = false;
        Serial.println("Could not create the ESD report file.");
        return;
      }
      file.println("date,time,identifier,name,result,temperature_c,humidity_rh");
      file.close();
    }
    refreshReportCount();
  }

  void refreshReportCount() {
    reportRecordCount_ = 0;
    if (!sdCardReady_) return;
    File file = SD.open(REPORT_PATH, FILE_READ);
    if (!file) return;
    bool header = true;
    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();
      if (header) {
        header = false;
      } else if (line.length()) {
        ++reportRecordCount_;
      }
    }
    file.close();
  }

  void deleteAllReportRecords() {
    reportDeleteConfirmation_ = false;
    reportOffset_ = 0;
    if (!sdCardReady_) {
      Serial.println("Report deletion failed: SD card is unavailable.");
      drawReport();
      return;
    }

    if (SD.exists(REPORT_PATH) && !SD.remove(REPORT_PATH)) {
      Serial.println("Report deletion failed: CSV file could not be removed.");
      drawReport();
      return;
    }

    initializeReport();
    Serial.println("All ESD report records deleted; CSV header recreated.");
    drawReport();
  }

  void appendCurrentTestRecord() {
    if (!sdCardReady_) {
      Serial.println("ESD record not saved: SD card is unavailable.");
      return;
    }

    char date[11];
    char time[9];
    snprintf(date, sizeof(date), "2026-%02ld-%02ld", random(1, 13),
             random(1, 29));
    snprintf(time, sizeof(time), "%02ld:%02ld:%02ld", random(0, 24),
             random(0, 60), random(0, 60));
    const char *result = finalPass_ ? "OK" :
                         (finalLow_ ? "LO" :
                          (finalHigh_ ? "HI" : "NOT_OK"));

    File file = SD.open(REPORT_PATH, FILE_APPEND);
    if (!file) {
      Serial.println("ESD record not saved: report file could not be opened.");
      return;
    }
    file.printf("%s,%s,%s,%s,%s,%.2f,%.2f\n", date, time,
                currentBadge_.length() ? currentBadge_.c_str() : "---",
                currentUser_.length() ? currentUser_.c_str() : "Unknown user",
                result, temperatureC_, humidity_);
    file.close();
    ++reportRecordCount_;
    Serial.printf("ESD record saved to %s: %s %s %s\n", REPORT_PATH, date,
                  time, result);
  }

  String reportField(const String &line, uint8_t wantedField) const {
    int start = 0;
    for (uint8_t field = 0; field < wantedField; ++field) {
      start = line.indexOf(',', start);
      if (start < 0) return "";
      ++start;
    }
    int end = line.indexOf(',', start);
    if (end < 0) end = line.length();
    return line.substring(start, end);
  }

  void saveCurrentUser(int16_t faceId) {
    int8_t index = findUserByBadge(currentBadge_.c_str());
    if (index < 0) {
      for (uint8_t candidate = 0; candidate < MAX_USERS; ++candidate) {
        if (!users_[candidate].used) {
          index = candidate;
          break;
        }
      }
    }
    if (index < 0) {
      Serial.println("User registry is full; record was not saved.");
      return;
    }

    UserRecord &user = users_[index];
    memset(&user, 0, sizeof(user));
    user.used = 1;
    user.faceId = faceId;
    currentBadge_.toCharArray(user.badge, sizeof(user.badge));
    firstName_.toCharArray(user.firstName, sizeof(user.firstName));
    lastName_.toCharArray(user.lastName, sizeof(user.lastName));
    persistRegistry();
    Serial.printf("Saved user %s %s with face ID %d.\n", user.firstName,
                  user.lastName, user.faceId);
  }

  void changeScreen(Screen next) {
    // Centralizing transitions here ensures every screen resets its timers and
    // test state in the same predictable way.
    screen_ = next;
    screenStartedMs_ = millis();
    if (next == Screen::FaceScan) recognizedFaceId_ = -1;
    touchesEnabledAfterMs_ = screenStartedMs_ + 600;
    testHolding_ = false;
    testEvaluated_ = false;
    if (next == Screen::Test) {
      finalLow_ = false;
      finalHigh_ = false;
      finalPass_ = false;
    }
    Serial.printf("HMI screen changed to state %u.\n", static_cast<uint8_t>(next));
    draw();
  }

  void clear(const char *title, uint16_t accent = BLUE) {
    display_.fillScreen(BACKGROUND);
    display_.setTextDatum(middle_center);
    display_.setTextColor(TFT_WHITE, BACKGROUND);
    display_.setTextSize(3);
    display_.drawString(title, 400, 48);
    display_.setTextSize(1);
    display_.setTextColor(MUTED, BACKGROUND);
    display_.drawString("ESD CONTROL STATION  |  INTERFACE DRAFT", 400, 82);
  }

  void button(int16_t x, int16_t y, int16_t width, int16_t height,
              const char *label, uint16_t color = BLUE, uint8_t textSize = 2) {
    display_.fillRoundRect(x, y, width, height, 14, color);
    display_.drawRoundRect(x, y, width, height, 14, TFT_WHITE);
    display_.setTextDatum(middle_center);
    display_.setTextColor(TFT_WHITE, color);
    display_.setTextSize(textSize);
    display_.drawString(label, x + width / 2, y + height / 2);
  }

  void text(const char *value, int16_t x, int16_t y, uint8_t size = 2,
            uint16_t color = TFT_WHITE) {
    display_.setTextDatum(middle_center);
    display_.setTextColor(color, BACKGROUND);
    display_.setTextSize(size);
    display_.drawString(value, x, y);
  }

  void draw() {
    // Only the function for the current screen is allowed to redraw the page.
    switch (screen_) {
      case Screen::Welcome: drawWelcome(); break;
      case Screen::FaceScan: drawFaceScan(); break;
      case Screen::BadgeScan: drawBadgeScan(); break;
      case Screen::Registration: drawRegistration(); break;
      case Screen::FaceConsent: drawFaceConsent(); break;
      case Screen::FaceEnrollment: drawFaceEnrollment(); break;
      case Screen::Test: drawTest(); break;
      case Screen::Failure: drawFailure(); break;
      case Screen::Thanks: drawThanks(); break;
      case Screen::Report: drawReport(); break;
      case Screen::StoredUsers: drawStoredUsers(); break;
      case Screen::Diagnostic: drawDiagnostic(); break;
    }
  }

  void drawWelcome() {
    clear("WELCOME");
    display_.fillCircle(400, 190, 62, BLUE);
    display_.drawCircle(400, 190, 63, TFT_WHITE);
    text("Touch START to begin", 400, 300, 2);
    button(285, 340, 230, 45, "START", BLUE, 2);
    button(285, 400, 230, 45, "REPORT", CARD, 2);
    button(25, 400, 230, 45, "STORED USERS", CARD, 1);
    button(545, 400, 230, 45, "DIAGNOSTIC", CARD, 1);
    drawFooter();
  }

  void drawDiagnosticIndicator(int16_t x, const char *label, bool active) {
    const uint16_t color = active ? GREEN : CARD;
    display_.fillRoundRect(x, 145, 160, 145, 16, color);
    display_.drawRoundRect(x, 145, 160, 145, 16,
                           active ? TFT_GREEN : MUTED);
    display_.setTextDatum(middle_center);
    display_.setTextColor(TFT_WHITE, color);
    display_.setTextSize(3);
    display_.drawString(label, x + 80, 190);
    display_.setTextSize(2);
    display_.drawString(active ? "HIGH" : "LOW", x + 80, 245);
  }

  void drawDiagnosticSignals() {
    drawDiagnosticIndicator(45, "LO", latestLow_);
    drawDiagnosticIndicator(225, "OK", latestPass_);
    drawDiagnosticIndicator(405, "HI", latestHigh_);
    drawDiagnosticIndicator(585, "BP", latestButtonPressed_);
  }

  void drawDiagnostic() {
    clear("TESTER DIAGNOSTIC", TFT_CYAN);
    text("Live signals received from the Secondary Controller", 400, 108, 1,
         MUTED);
    drawDiagnosticSignals();
    text("BP = tester button pressed", 400, 330, 1, TFT_CYAN);
    button(300, 375, 200, 55, "BACK", CARD, 1);
    drawFooter();
  }

  void drawFaceScan() {
    clear("FACIAL IDENTIFICATION");
    display_.drawRoundRect(300, 120, 200, 185, 20, TFT_CYAN);
    display_.drawCircle(400, 180, 34, TFT_CYAN);
    display_.drawRoundRect(345, 225, 110, 55, 25, TFT_CYAN);
    text("Scanning your face...", 400, 325, 2, TFT_CYAN);
    text("Please look directly at the camera", 400, 350, 1);
    button(275, 365, 250, 55, "Simulate recognized", CARD, 1);
    drawFooter();
  }

  void drawBadgeScan() {
    clear("BADGE IDENTIFICATION");
    text("Face not recognized", 400, 145, 2, TFT_ORANGE);
    display_.fillRoundRect(280, 185, 240, 105, 16, CARD);
    display_.drawRoundRect(280, 185, 240, 105, 16, TFT_WHITE);
    text("SCAN BADGE", 400, 238, 3);
    text("Present your badge to the reader", 400, 330, 2);
    button(275, 365, 250, 55, "Simulate badge", CARD, 1);
    drawFooter();
  }

  void drawRegistration() {
    clear("NEW USER REGISTRATION", TFT_CYAN);
    String badgeLabel = "Badge ID: " + currentBadge_;
    text(badgeLabel.c_str(), 400, 108, 1, TFT_YELLOW);
    drawField(35, 126, 350, 50, "First name", firstName_, activeField_ == 0);
    drawField(415, 126, 350, 50, "Last name", lastName_, activeField_ == 1);

    const char *rows[] = {"QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
    const uint8_t lengths[] = {10, 9, 7};
    const int16_t starts[] = {40, 76, 148};
    for (uint8_t row = 0; row < 3; ++row) {
      const int16_t y = 195 + row * 55;
      for (uint8_t key = 0; key < lengths[row]; ++key) {
        char label[2] = {rows[row][key], '\0'};
        button(starts[row] + key * 72, y, 64, 46, label, CARD, 2);
      }
    }
    button(80, 365, 330, 55, "SPACE", CARD);
    button(425, 365, 135, 55, "BACK", CARD, 1);
    button(575, 365, 145, 55, "SAVE", GREEN, 2);
    drawFooter();
  }

  void drawField(int16_t x, int16_t y, int16_t width, int16_t height,
                 const char *label, const String &value, bool active) {
    const uint16_t border = active ? TFT_CYAN : MUTED;
    display_.fillRoundRect(x, y, width, height, 10, CARD);
    display_.drawRoundRect(x, y, width, height, 10, border);
    display_.setTextDatum(top_left);
    display_.setTextSize(1);
    display_.setTextColor(border, CARD);
    display_.drawString(label, x + 12, y + 6);
    display_.setTextSize(2);
    display_.setTextColor(TFT_WHITE, CARD);
    display_.drawString(value.length() ? value.c_str() : "Tap and type",
                        x + 12, y + 25);
  }

  void handleRegistrationTouch(uint16_t x, uint16_t y) {
    if (inside(x, y, 35, 126, 350, 50)) {
      activeField_ = 0;
      drawRegistration();
      return;
    }
    if (inside(x, y, 415, 126, 350, 50)) {
      activeField_ = 1;
      drawRegistration();
      return;
    }

    String &field = activeField_ == 0 ? firstName_ : lastName_;
    const char *rows[] = {"QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
    const uint8_t lengths[] = {10, 9, 7};
    const int16_t starts[] = {40, 76, 148};
    for (uint8_t row = 0; row < 3; ++row) {
      const int16_t keyY = 195 + row * 55;
      for (uint8_t key = 0; key < lengths[row]; ++key) {
        if (inside(x, y, starts[row] + key * 72, keyY, 64, 46)) {
          if (field.length() < 20) {
            char letter = rows[row][key];
            if (field.length() > 0) letter = tolower(letter);
            field += letter;
          }
          drawRegistration();
          return;
        }
      }
    }

    if (inside(x, y, 80, 365, 330, 55) && field.length() < 20) {
      field += ' ';
      drawRegistration();
    } else if (inside(x, y, 425, 365, 135, 55) && field.length()) {
      field.remove(field.length() - 1);
      drawRegistration();
    } else if (inside(x, y, 575, 365, 145, 55) && firstName_.length() &&
               lastName_.length()) {
      currentUser_ = firstName_ + " " + lastName_;
      changeScreen(Screen::FaceConsent);
    }
  }

  void drawFaceConsent() {
    clear("FACIAL REGISTRATION", TFT_CYAN);
    text((firstName_ + " " + lastName_).c_str(), 400, 135, 2, TFT_CYAN);
    text("Would you like to register your face?", 400, 195, 2);
    text("This allows identification without scanning your badge.", 400, 225,
         1, MUTED);
    button(150, 245, 220, 90, "YES", GREEN, 3);
    button(430, 245, 220, 90, "NO", RED, 3);
    drawFooter();
  }

  void drawFaceEnrollment() {
    clear("REGISTER YOUR FACE", TFT_CYAN);
    display_.drawRoundRect(300, 120, 200, 185, 20, TFT_CYAN);
    display_.drawCircle(400, 180, 34, TFT_CYAN);
    display_.drawRoundRect(345, 225, 110, 55, 25, TFT_CYAN);
    if (enrollmentFailed_) {
      text("Face registration failed", 400, 325, 2, TFT_RED);
      button(150, 360, 220, 60, "TRY AGAIN", BLUE, 1);
      button(430, 360, 220, 60, "SKIP", CARD, 1);
    } else {
      text("Look directly at the camera", 400, 330, 2, TFT_CYAN);
      text("Please remain still while your face is saved", 400, 365, 1);
    }
    drawFooter();
  }

  void drawTest() {
    clear("ESD WRIST-STRAP TEST", TFT_CYAN);
    text(currentUser_.length() ? currentUser_.c_str() : "Identified user", 400,
         115, 2, TFT_CYAN);
    text("1. Insert the banana plug into the station", 400, 175, 2);
    text("2. Press and hold the tester button for 2 seconds", 400, 220, 2);
    display_.drawRoundRect(150, 270, 500, 45, 12, MUTED);
    text("Waiting for test...", 400, 293, 2, MUTED);
    button(185, 365, 200, 55, "Demo PASS", GREEN, 1);
    button(415, 365, 200, 55, "Demo FAIL", RED, 1);
    drawFooter();
  }

  void drawTestProgress(uint32_t heldMs) {
    const uint32_t limited = heldMs > TEST_SAMPLE_DELAY_MS
                                 ? TEST_SAMPLE_DELAY_MS
                                 : heldMs;
    const int16_t progressWidth = static_cast<int16_t>(
        (limited * 496) / TEST_SAMPLE_DELAY_MS);
    display_.fillRoundRect(152, 272, 496, 41, 10, CARD);
    if (progressWidth > 0) {
      display_.fillRoundRect(152, 272, progressWidth, 41, 10, BLUE);
    }
    char message[32];
    snprintf(message, sizeof(message), "Sampling... %.1f / 2.0 s",
             limited / 1000.0f);
    display_.setTextDatum(middle_center);
    display_.setTextColor(TFT_WHITE, limited ? BLUE : CARD);
    display_.setTextSize(2);
    display_.drawString(message, 400, 293);
  }

  void drawFailure() {
    clear("TEST NOT PASSED", RED);
    display_.fillCircle(400, 145, 48, RED);
    text("!", 400, 145, 4);
    const char *reason = finalLow_ ? "Result: LOW" :
                         (finalHigh_ ? "Result: HIGH" : "Result: NOT OK");
    text(reason, 400, 215, 3, TFT_ORANGE);
    text("Try again, adjust or replace the wrist strap,", 400, 275, 2);
    text("or moisten the skin if necessary.", 400, 310, 2);
    button(265, 375, 270, 60, "TRY AGAIN", BLUE, 2);
    drawFooter();
  }

  void drawThanks() {
    clear("TEST PASSED", GREEN);
    display_.fillCircle(400, 145, 52, GREEN);
    text("OK", 400, 145, 3);
    text("Thank you!", 400, 225, 3, TFT_GREEN);
    text("You are ready to start working at the bench!", 400, 275, 2);
    text("Returning to the welcome screen...", 400, 330, 1, MUTED);
    button(275, 365, 250, 55, "Finish now", CARD, 1);
    drawFooter();
  }

  void drawReport() {
    clear("ESD TEST REPORT", TFT_CYAN);
    display_.setTextDatum(top_left);
    display_.setTextSize(1);
    display_.setTextColor(TFT_CYAN, BACKGROUND);
    display_.drawString("DATE", 15, 104);
    display_.drawString("TIME", 92, 104);
    display_.drawString("IDENTIFIER", 145, 104);
    display_.drawString("NAME", 235, 104);
    display_.drawString("RESULT", 465, 104);
    display_.drawString("TEMP C", 550, 104);
    display_.drawString("RH %", 655, 104);
    display_.drawFastHLine(12, 122, 776, MUTED);

    if (!sdCardReady_) {
      text("SD card unavailable", 400, 240, 2, TFT_ORANGE);
    } else if (reportRecordCount_ == 0) {
      text("No ESD test records yet", 400, 240, 2, MUTED);
    } else {
      File file = SD.open(REPORT_PATH, FILE_READ);
      uint16_t recordIndex = 0;
      uint8_t visibleRow = 0;
      bool header = true;
      while (file && file.available() && visibleRow < REPORT_ROWS) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (header) {
          header = false;
          continue;
        }
        if (!line.length()) continue;
        if (recordIndex++ < reportOffset_) continue;

        const int16_t y = 137 + visibleRow * 34;
        if (visibleRow % 2 == 0) display_.fillRect(10, y - 6, 780, 30, CARD);
        display_.setTextColor(TFT_WHITE,
                              visibleRow % 2 == 0 ? CARD : BACKGROUND);
        display_.drawString(reportField(line, 0).substring(5).c_str(), 15, y);
        display_.drawString(reportField(line, 1).substring(0, 5).c_str(), 92, y);
        display_.drawString(reportField(line, 2).c_str(), 145, y);
        String name = reportField(line, 3);
        if (name.length() > 25) name = name.substring(0, 25);
        display_.drawString(name.c_str(), 235, y);
        display_.drawString(reportField(line, 4).c_str(), 465, y);
        display_.drawString(reportField(line, 5).c_str(), 550, y);
        display_.drawString(reportField(line, 6).c_str(), 655, y);
        ++visibleRow;
      }
      if (file) file.close();
    }

    char countText[48];
    snprintf(countText, sizeof(countText), "Rows %u-%u of %u",
             reportRecordCount_ ? reportOffset_ + 1 : 0,
             min<uint16_t>(reportOffset_ + REPORT_ROWS, reportRecordCount_),
             reportRecordCount_);
    text(countText, 690, 370, 1, MUTED);
    button(35, 390, 170, 45, "BACK", CARD, 1);
    button(250, 390, 130, 45, "UP", reportOffset_ ? BLUE : CARD, 1);
    button(420, 390, 130, 45, "DOWN",
           reportOffset_ + REPORT_ROWS < reportRecordCount_ ? BLUE : CARD, 1);
    if (reportRecordCount_ > 0) {
      button(590, 390, 180, 45,
             reportDeleteConfirmation_ ? "CONFIRM DELETE" : "DELETE ALL",
             reportDeleteConfirmation_ ? RED : CARD, 1);
    }
    drawFooter();
  }

  void drawStoredUsers() {
    clear("STORED USERS", TFT_CYAN);
    display_.setTextDatum(top_left);
    display_.setTextSize(1);
    display_.setTextColor(TFT_CYAN, BACKGROUND);
    display_.drawString("ID", 30, 108);
    display_.drawString("NAME", 285, 108);
    display_.drawString("FACIAL ID STORED", 610, 108);
    display_.drawFastHLine(20, 126, 760, MUTED);

    uint8_t visibleRow = 0;
    for (uint8_t index = 0; index < MAX_USERS; ++index) {
      if (!users_[index].used) continue;
      const int16_t y = 143 + visibleRow * 34;
      if (visibleRow % 2 == 0) display_.fillRect(20, y - 6, 760, 30, CARD);
      display_.setTextColor(TFT_WHITE,
                            visibleRow % 2 == 0 ? CARD : BACKGROUND);
      String badge = users_[index].badge;
      if (badge.length() > 32) badge = badge.substring(0, 32);
      display_.drawString(badge.c_str(), 30, y);
      String name = userFullName(users_[index]);
      if (name.length() > 28) name = name.substring(0, 28);
      display_.drawString(name.c_str(), 285, y);
      char faceLabel[18];
      if (users_[index].faceId >= 0) {
        snprintf(faceLabel, sizeof(faceLabel), "YES (ID %d)", users_[index].faceId);
      } else {
        snprintf(faceLabel, sizeof(faceLabel), "NO");
      }
      display_.drawString(faceLabel, 610, y);
      ++visibleRow;
    }

    if (visibleRow == 0) text("No users stored", 400, 240, 2, MUTED);
    char countLabel[24];
    snprintf(countLabel, sizeof(countLabel), "%u of %u users", visibleRow,
             MAX_USERS);
    text(countLabel, 400, 365, 1, MUTED);
    button(110, 390, 220, 45, "BACK", CARD, 1);
    if (visibleRow > 0) {
      button(470, 390, 220, 45,
             userDeleteConfirmation_ ? "CONFIRM DELETE" : "DELETE ALL USERS",
             userDeleteConfirmation_ ? RED : CARD, 1);
    }
    drawFooter();
  }

  void drawFooter() {
    display_.fillRect(0, 448, 800, 32, CARD);
    display_.setTextDatum(middle_center);
    display_.setTextSize(1);
    display_.setTextColor(MUTED, CARD);
    char footer[90];
    if (isnan(temperatureC_) || isnan(humidity_)) {
      snprintf(footer, sizeof(footer), "Temperature: -- C     Humidity: -- %%RH");
    } else {
      snprintf(footer, sizeof(footer), "Temperature: %.1f C     Humidity: %.1f %%RH",
               temperatureC_, humidity_);
    }
    display_.drawString(footer, 400, 464);
  }
};
