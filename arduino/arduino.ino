#include <Wire.h>
#include <EEPROM.h>
#include "IranGSM.h"
#include <string.h>

#ifdef __AVR__
  #include <avr/wdt.h>
  #define WDT_AVAILABLE 1
#else
  #define WDT_AVAILABLE 0
#endif

// === HW pins ===
#define RX_PIN A2
#define TX_PIN A1
#define PIN_HALL 2

// LED mapping per user: RED = D6, GREEN = D7, BLUE = D5
#define LED_R 7
#define LED_G 6
#define LED_B 5

// Optional: control GSM power (set to -1 to disable)
#define GSM_PWR_PIN -1
#define GSM_PWR_ACTIVE_HIGH true

// === External EEPROM ===
#define EEPROM_I2C_ADDR 0x50
#define EEPROM_TOTAL_BYTES 32768UL
#define EEPROM_PAGE_SIZE 64
#define RECORD_SIZE 8
#define RECORDS_COUNT (EEPROM_TOTAL_BYTES / RECORD_SIZE)
#define COMMIT_HI 0xA5
#define COMMIT_LO 0x5A

// === Internal EEPROM layout ===
#define IEEP_PPL_ADDR          0   // uint16_t (2 bytes) -> pulses per liter

// Tunables
#define EEPROM_NUM_MAXCHUNK    140
#define DEFAULT_PULSES_PER_LITER 670U
#define LITERS_PER_SAVE 5

// Global IranGSM instance (as you had it)
IranGSM myGSM(RX_PIN, TX_PIN);

// GSM readiness tuning (kept for run-time checks)
const unsigned long GSM_CHECK_INTERVAL_MS = 2000UL;
const unsigned long GSM_STABLE_DELAY_MS   = 3000UL;
const uint8_t GSM_CONSECUTIVE_SUCCESS     = 3;
const int GSM_MIN_RSSI                    = 5; // set -1 to accept >=0

// Startup simplification params
const unsigned long STARTUP_RED_MS = 15000UL; // wait 15s red on boot
const unsigned long AT_RETRY_MS = 5000UL;     // wait 5s between AT+CREG? retries
const int POWER_CYCLE_AFTER_ATTEMPTS = 4;     // attempt power-cycle after this many failed probes

const bool DEBUG = true;
const String TRUSTED_NUMBER = "+989355690823";

// runtime counters
volatile unsigned int pulseCounter = 0;
volatile unsigned int literCounter = 0;
unsigned int lastSavedLiters = 0;

uint32_t lastSequence = 0;
uint32_t eepromWriteIndex = 0;
volatile uint16_t pulsesPerLiterVar = DEFAULT_PULSES_PER_LITER;

#define SMS_Q_SZ 5
struct SmsJob { bool used=false; String to; String txt; };
SmsJob smsQ[SMS_Q_SZ];

volatile bool saveRequested = false;
volatile uint16_t saveRequestedLiters = 0;

bool waitingForInitialLiters = true;
bool gsmReady = false;
unsigned long lastGsmCheck = 0;
bool ledBlinkState = false;

uint8_t gsmSuccessCount = 0;
unsigned long gsmCandidateSince = 0;

volatile bool startSent = false;
bool hallAttached = false;

// Watchdog state
bool watchdogEnabled = false;
const int WDT_TIMEOUT_SECONDS = 8; // used for debug output; actual period uses WDTO_8S

// If using common-anode LEDs (HIGH -> off), set true
const bool COMMON_ANODE = false;

// ---------- helpers ----------
void setLED_raw(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}
void setLED(bool r, bool g, bool b) {
  if (COMMON_ANODE) setLED_raw(!r, !g, !b);
  else setLED_raw(r,g,b);
}

// Watchdog helpers (AVR only)
void enableWatchdogIfNeeded() {
#if WDT_AVAILABLE
  if (!watchdogEnabled) {
    // enable 8s watchdog
    wdt_enable(WDTO_8S);
    watchdogEnabled = true;
    if (DEBUG) {
      Serial.print(F("Watchdog enabled (")); Serial.print(WDT_TIMEOUT_SECONDS); Serial.println(F("s)"));
    }
  }
#endif
}
void disableWatchdogIfNeeded() {
#if WDT_AVAILABLE
  if (watchdogEnabled) {
    wdt_disable();
    watchdogEnabled = false;
    if (DEBUG) Serial.println(F("Watchdog disabled"));
  }
#endif
}
inline void feedWatchdog() {
#if WDT_AVAILABLE
  wdt_reset();
#endif
}

// pulse ISR (fast)
void countPulse() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();
  if (now - lastMicros < 2000UL) return; // debounce ~2ms
  lastMicros = now;

  pulseCounter++;
  if (pulseCounter >= pulsesPerLiterVar) {
    pulseCounter = 0;
    literCounter++;
  }
}

// ---------- external EEPROM (unchanged logic, but we feed WDT inside loops) ----------
void eeprom_wait_ready() {
  while (true) {
    Wire.beginTransmission(EEPROM_I2C_ADDR);
    uint8_t e = Wire.endTransmission();
    if (e == 0) break;
    // feed WDT while waiting
    feedWatchdog();
    delayMicroseconds(500);
  }
}

void eeprom_write_pagewise(uint16_t addr, const uint8_t* data, uint16_t len) {
  uint16_t remaining = len;
  uint16_t offset = 0;
  while (remaining > 0) {
    uint16_t pageSpace = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
    uint16_t chunk = (remaining < pageSpace) ? remaining : pageSpace;
    while (chunk > 0) {
      uint8_t sub = (chunk > 30) ? 30 : chunk;
      Wire.beginTransmission(EEPROM_I2C_ADDR);
      Wire.write((uint8_t)(addr >> 8));
      Wire.write((uint8_t)(addr & 0xFF));
      for (uint8_t i = 0; i < sub; i++) {
        Wire.write(data[offset + i]);
        // occasional WDT feed for very long writes (rare)
        if ((i & 15) == 0) feedWatchdog();
      }
      Wire.endTransmission();
      eeprom_wait_ready();
      addr += sub; offset += sub; remaining -= sub; chunk -= sub;
    }
    // feed watchdog between page chunks
    feedWatchdog();
  }
}

void eeprom_read_burst(uint16_t addr, uint8_t* out, uint16_t len) {
  while (len > 0) {
    uint8_t chunk = (len > 30) ? 30 : len;
    Wire.beginTransmission(EEPROM_I2C_ADDR);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)EEPROM_I2C_ADDR, chunk);
    uint8_t i = 0;
    while (Wire.available() && i < chunk) {
      out[i++] = Wire.read();
      if ((i & 15) == 0) feedWatchdog();
    }
    addr += chunk; out += chunk; len -= chunk;
    feedWatchdog();
  }
}

inline uint16_t recAddr(uint32_t recIndex) {
  recIndex %= RECORDS_COUNT;
  return (uint16_t)(recIndex * RECORD_SIZE);
}

void eeprom_write_record(uint32_t recIndex, uint32_t seq, uint16_t liters) {
  uint8_t rec[RECORD_SIZE];
  rec[0] = (uint8_t)(seq >> 24); rec[1] = (uint8_t)(seq >> 16);
  rec[2] = (uint8_t)(seq >> 8);  rec[3] = (uint8_t)(seq);
  rec[4] = (uint8_t)(liters >> 8); rec[5] = (uint8_t)(liters);
  rec[6] = 0xFF; rec[7] = 0xFF;

  uint16_t addr = recAddr(recIndex);
  eeprom_write_pagewise(addr, rec, 6);

  rec[6] = COMMIT_HI; rec[7] = COMMIT_LO;
  eeprom_write_pagewise(addr + 6, rec + 6, 2);
}

bool eeprom_read_record(uint32_t recIndex, uint32_t &seq, uint16_t &liters, bool &valid) {
  uint8_t rec[RECORD_SIZE];
  eeprom_read_burst(recAddr(recIndex), rec, RECORD_SIZE);
  valid = (rec[6] == COMMIT_HI && rec[7] == COMMIT_LO);
  if (!valid) { seq = 0; liters = 0; return false; }
  seq = ((uint32_t)rec[0] << 24) | ((uint32_t)rec[1] << 16) | ((uint32_t)rec[2] << 8) | rec[3];
  liters = ((uint16_t)rec[4] << 8) | rec[5];
  return true;
}

uint16_t loadLitersFromEEPROM() {
  uint32_t bestSeq = 0; uint16_t bestLit = 0; uint32_t bestIdx = 0; bool any = false; uint8_t rec[RECORD_SIZE];
  for (uint32_t i = 0; i < RECORDS_COUNT; i++) {
    eeprom_read_burst(recAddr(i), rec, RECORD_SIZE);
    if (rec[6] == COMMIT_HI && rec[7] == COMMIT_LO) {
      uint32_t seq = ((uint32_t)rec[0] << 24) | ((uint32_t)rec[1] << 16) | ((uint32_t)rec[2] << 8) | rec[3];
      uint16_t lit = ((uint16_t)rec[4] << 8) | rec[5];
      if (!any || seq > bestSeq) { any = true; bestSeq = seq; bestLit = lit; bestIdx = i; }
    }
    feedWatchdog();
  }
  if (!any) {
    eepromWriteIndex = 0; lastSequence = 0;
    if (DEBUG) Serial.println(F("External EEPROM empty; start at 0 liters"));
    return 0;
  }
  eepromWriteIndex = bestIdx + 1;
  if (eepromWriteIndex >= RECORDS_COUNT) eepromWriteIndex = 0;
  lastSequence = bestSeq;
  if (DEBUG) {
    Serial.print(F("External EEPROM loaded liters=")); Serial.print(bestLit);
    Serial.print(F(" seq=")); Serial.print(bestSeq);
    Serial.print(F(" lastIdx=")); Serial.println(bestIdx);
  }
  return bestLit;
}

void saveLitersToEEPROM(uint16_t liters) {
  lastSequence++; if (lastSequence == 0) lastSequence = 1;
  eeprom_write_record(eepromWriteIndex, lastSequence, liters);
  if (DEBUG) {
    Serial.print(F("Saved liters to external EEPROM=")); Serial.print(liters);
    Serial.print(F(" seq=")); Serial.print(lastSequence);
    Serial.print(F(" idx=")); Serial.println(eepromWriteIndex);
  }
  eepromWriteIndex++; if (eepromWriteIndex >= RECORDS_COUNT) eepromWriteIndex = 0;
}

// ---------- SMS queue ----------
void enqueueSms(const String &to, const String &txt) {
  for (int i = 0; i < SMS_Q_SZ; i++) {
    if (!smsQ[i].used) {
      smsQ[i].used = true;
      smsQ[i].to = to;
      smsQ[i].txt = txt;
      if (DEBUG) { Serial.print(F("Enqueue SMS -> ")); Serial.print(to); Serial.print(F(" : ")); Serial.println(txt); }
      return;
    }
  }
  if (DEBUG) Serial.println(F("SMS queue full; dropping message"));
}

void serviceSmsOut() {
  for (int i = 0; i < SMS_Q_SZ; i++) {
    if (smsQ[i].used) {
      if (DEBUG) { Serial.print(F("Sending (IranGSM) -> ")); Serial.print(smsQ[i].to); Serial.print(F(" : ")); Serial.println(smsQ[i].txt); }
      myGSM.SMS_Send(smsQ[i].to, smsQ[i].txt);

      // If this was the START message to the main trusted number, mark that START was sent
      if (!startSent) {
        String toNorm = smsQ[i].to;
        toNorm.replace(" ", ""); toNorm.replace("(", ""); toNorm.replace(")", "");
        String tNorm = TRUSTED_NUMBER;
        tNorm.replace(" ", ""); tNorm.replace("(", ""); tNorm.replace(")", "");
        if (smsQ[i].txt == "START" && toNorm == tNorm) {
          startSent = true;
          setLED(false, true, false); // green
          if (DEBUG) Serial.println(F("START SMS actually sent -> LED GREEN"));

          // Enable watchdog now that START is actually sent
          enableWatchdogIfNeeded();
        }
      }

      smsQ[i].used = false;
      smsQ[i].to = ""; smsQ[i].txt = "";
      if (DEBUG) Serial.println(F("SMS Sent (IranGSM)"));
      return; // send one SMS per serviceSmsOut call
    }
  }
}

// Request save: only short critical section to copy volatile
void requestSave(uint16_t liters) {
  noInterrupts();
  saveRequestedLiters = liters;
  saveRequested = true;
  interrupts();
}

// ---------- Internal EEPROM helpers ----------
String trimNumberFormatting(const String &src) {
  String s;
  for (uint16_t i = 0; i < src.length(); ++i) {
    char c = src[i];
    if (c == '+' || isDigit(c)) s += c;
  }
  return s;
}

String digitsOnly(const String &src) {
  String s;
  for (uint16_t i = 0; i < src.length(); ++i) if (isDigit(src[i])) s += src[i];
  return s;
}

String tailDigits(const String &src, uint8_t tailLen = 11) {
  String d = digitsOnly(src);
  if (d.length() <= tailLen) return d;
  return d.substring(d.length() - tailLen);
}

uint16_t iEepromGetPPL() {
  uint8_t lo = EEPROM.read(IEEP_PPL_ADDR);
  uint8_t hi = EEPROM.read(IEEP_PPL_ADDR + 1);
  uint16_t val = ((uint16_t)hi << 8) | lo;
  if (val == 0xFFFF || val == 0) return DEFAULT_PULSES_PER_LITER;
  return val;
}

void iEepromSetPPL(uint16_t val) {
  if (val == 0) return;
  uint8_t lo = (uint8_t)(val & 0xFF);
  uint8_t hi = (uint8_t)(val >> 8);
  EEPROM.update(IEEP_PPL_ADDR, lo);
  EEPROM.update(IEEP_PPL_ADDR + 1, hi);
  pulsesPerLiterVar = val;
  if (DEBUG) {
    Serial.print(F("PPL updated in internal EEPROM: "));
    Serial.println(pulsesPerLiterVar);
  }
}

void iEepromInitIfNeeded() {
  uint8_t lo = EEPROM.read(IEEP_PPL_ADDR);
  uint8_t hi = EEPROM.read(IEEP_PPL_ADDR + 1);
  uint16_t val = ((uint16_t)hi << 8) | lo;
  if (val == 0xFFFF || val == 0) {
    if (DEBUG) Serial.println(F("Internal EEPROM PPL uninitialized; writing default PPL"));
    uint16_t defaultPpl = DEFAULT_PULSES_PER_LITER;
    uint8_t dlo = (uint8_t)(defaultPpl & 0xFF);
    uint8_t dhi = (uint8_t)(defaultPpl >> 8);
    EEPROM.update(IEEP_PPL_ADDR, dlo);
    EEPROM.update(IEEP_PPL_ADDR + 1, dhi);
    delay(10);
  }
  pulsesPerLiterVar = iEepromGetPPL();
  if (DEBUG) {
    Serial.print(F("Loaded PPL: "));
    Serial.println(pulsesPerLiterVar);
  }
}

// ---------- trusted checks ----------
bool isSenderMainTrusted(const String &fromRaw) {
  String f = fromRaw; f.replace(" ", ""); f.replace("(", ""); f.replace(")", "");
  String t = TRUSTED_NUMBER; t.replace(" ", ""); t.replace("(", ""); t.replace(")", "");
  int tailLen = (t.length() >= 11) ? 11 : t.length();
  String tTail = t.substring(t.length() - tailLen);
  String fTail = (f.length() > tailLen) ? f.substring(f.length() - tailLen) : f;
  return fTail == tTail;
}

// ---------- SMS helpers ----------
void sendSmsChunked(const String &to, const String &text) {
  int start = 0;
  int len = text.length();
  while (start < len) {
    int remain = len - start;
    int take = (remain > EEPROM_NUM_MAXCHUNK) ? EEPROM_NUM_MAXCHUNK : remain;
    if (take == EEPROM_NUM_MAXCHUNK) {
      int cut = -1;
      for (int i = start + take - 1; i > start; --i) {
        char c = text[i];
        if (c == ',' || c == ' ') { cut = i; break; }
      }
      if (cut > start) take = cut - start;
    }
    String piece = text.substring(start, start + take);
    enqueueSms(to, piece);
    start += take;
    while (start < len && (text[start] == ' ' || text[start] == ',')) start++;
  }
}

// ---------- GSM helpers ----------
int getGsmRssi() {
  String resp = myGSM.sendAT("AT+CSQ", 2000);
  int idx = resp.indexOf("+CSQ:");
  if (idx == -1) return -1;
  int comma = resp.indexOf(',', idx);
  if (comma == -1) return -1;
  int rssi = resp.substring(idx + 6, comma).toInt();
  if (rssi == 99) return -1;
  return rssi; // 0..31
}

void sendStartToTrustedNumbers() {
  enqueueSms(TRUSTED_NUMBER, "START");
  if (DEBUG) {
    Serial.print(F("Queued START to main trusted: "));
    Serial.println(TRUSTED_NUMBER);
  }
}

// Probe registration with AT+CREG? — returns true if stat==1 or stat==5 (registered home or roaming)
bool probeGsmRegistration() {
  String r = myGSM.sendAT("AT+CREG?", 2000);
  if (DEBUG) {
    Serial.print(F("AT+CREG? -> '")); Serial.print(r); Serial.println(F("'"));
  }
  int idx = r.indexOf("+CREG:");
  if (idx == -1) {
    // fallback: check plain AT OK (module alive) but not registered
    if (r.indexOf("OK") != -1) {
      if (DEBUG) Serial.println(F("AT OK but no +CREG response"));
    }
    return false;
  }

  // parse stat after comma
  int comma = r.indexOf(',', idx);
  int pos = (comma >= 0) ? comma + 1 : idx + 6;
  while (pos < (int)r.length() && !isDigit(r[pos])) pos++;
  if (pos >= (int)r.length()) return false;
  String num;
  while (pos < (int)r.length() && isDigit(r[pos])) { num += r[pos]; pos++; }
  if (num.length() == 0) return false;
  int stat = num.toInt();
  if (DEBUG) {
    Serial.print(F("Parsed CREG stat=")); Serial.println(stat);
  }
  return (stat == 1 || stat == 5);
}

// Attempt a soft power-cycle (if GSM_PWR_PIN wired)
void softPowerCycleGsm() {
  if (GSM_PWR_PIN < 0) return;
  if (DEBUG) Serial.println(F("Soft power-cycle of GSM: toggling GSM_PWR_PIN"));
  if (GSM_PWR_ACTIVE_HIGH) {
    digitalWrite(GSM_PWR_PIN, LOW); delay(700);
    digitalWrite(GSM_PWR_PIN, HIGH); delay(1500);
  } else {
    digitalWrite(GSM_PWR_PIN, HIGH); delay(700);
    digitalWrite(GSM_PWR_PIN, LOW); delay(1500);
  }
}

// ---------- incoming SMS processing ----------
void processIncomingSms(const String &fromRaw, const String &bodyRaw) {
  if (DEBUG) {
    Serial.print(F("SMS FROM: ")); Serial.println(fromRaw);
    Serial.print(F("BODY: ")); Serial.println(bodyRaw);
  }

  String from = fromRaw;
  String fromClean = from; fromClean.replace(" ", ""); fromClean.replace("(", ""); fromClean.replace(")", "");
  String body = bodyRaw; body.trim();
  String up = body; up.toUpperCase();

  if (!isSenderMainTrusted(fromClean)) {
    if (DEBUG) Serial.println(F("Ignoring unauthorized sender"));
    return;
  }

  // ADMIN commands
  if (up.startsWith("SET PPL ") || up.startsWith("SET PULSES ")) {
    String arg = (up.startsWith("SET PPL ")) ? body.substring(8) : body.substring(11);
    arg.trim();
    bool allDigits = true;
    for (uint16_t i=0;i<arg.length() && allDigits;i++) if (!isDigit(arg[i])) allDigits=false;
    if (!allDigits || arg.length()==0) { enqueueSms(from, "INVALID PPL"); return; }
    uint16_t val = (uint16_t)arg.toInt();
    if (val == 0) { enqueueSms(from, "INVALID PPL"); return; }
    iEepromSetPPL(val);
    enqueueSms(from, "PPL SET TO " + String(val));
    return;
  }

  if (up == "GET NUMBERS" || up == "GET LIST" || up == "NUMBERS") {
    String reply = "MAIN:" + TRUSTED_NUMBER;
    sendSmsChunked(from, reply);
    return;
  }

  if (up == "GET PPL" || up == "GET PULSES") {
    uint16_t cur = iEepromGetPPL();
    enqueueSms(from, "PPL=" + String(cur));
    return;
  }

  // Initial liters input
  if (waitingForInitialLiters) {
    bool allDigits = true;
    String tmp = body; tmp.trim();
    if (tmp.length() == 0) allDigits = false;
    for (uint16_t i=0;i<tmp.length() && allDigits;i++) if (!isDigit(tmp[i])) allDigits=false;
    if (allDigits) {
      uint16_t v = (uint16_t) tmp.toInt();
      noInterrupts();
      literCounter = v;
      interrupts();
      waitingForInitialLiters = false;
      enqueueSms(from, "LITER SET TO " + String(v));
      requestSave(v);
      return;
    }
  }

  if (up == "GET") {
    uint16_t cur;
    noInterrupts(); cur = literCounter; interrupts();
    enqueueSms(from, "LITERS=" + String(cur));
    return;
  }

  if (up == "RESET") {
    noInterrupts(); literCounter = 0; interrupts();
    enqueueSms(from, "LITERS RESET");
    requestSave(0);
    return;
  }

  if (up.startsWith("POST ")) {
    String num = body.substring(5); num.trim();
    bool allDigits = true;
    if (num.length() == 0) allDigits = false;
    for (uint16_t i=0;i<num.length() && allDigits;i++) if (!isDigit(num[i])) allDigits=false;
    if (allDigits) {
      uint16_t v = (uint16_t) num.toInt();
      noInterrupts(); literCounter = v; interrupts();
      enqueueSms(from, "LITERS SET TO " + String(v));
      requestSave(v);
    } else {
      enqueueSms(from, "INVALID NUMBER");
    }
    return;
  }

  if (up == "I SENT" || up == "SENT" || body.indexOf("فرست") != -1 || body.indexOf("فرستادم") != -1) {
    enqueueSms(from, "SAVED_LITERS=" + String(lastSavedLiters));
    return;
  }

  enqueueSms(from, "INVALID COMMAND");
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(9600);
  delay(2000); // ensure serial ready

  // Ensure watchdog disabled at start (if present)
#if WDT_AVAILABLE
  wdt_disable();
  watchdogEnabled = false;
#endif

  // Minimal early init only (no Wire, no interrupts)
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(PIN_HALL, INPUT_PULLUP);
  hallAttached = false;

  if (GSM_PWR_PIN >= 0) {
    pinMode(GSM_PWR_PIN, OUTPUT);
    if (GSM_PWR_ACTIVE_HIGH) digitalWrite(GSM_PWR_PIN, HIGH);
    else digitalWrite(GSM_PWR_PIN, LOW);
    if (DEBUG) {
      Serial.print(F("GSM_PWR_PIN active configured on D")); Serial.println(GSM_PWR_PIN);
    }
  }

  // Start RED and log
  setLED(true, false, false);
  if (DEBUG) {
    Serial.print(F("LED mapping: RED=D")); Serial.print(LED_R);
    Serial.print(F(" GREEN=D")); Serial.print(LED_G);
    Serial.print(F(" BLUE=D")); Serial.println(LED_B);
    Serial.println(F("Startup: RED for boot wait, then poll AT+CREG? until registered."));
    Serial.print(F("Initial wait ms: ")); Serial.println(STARTUP_RED_MS);
  }

  // provide a short settle time for power rails
  delay(STARTUP_RED_MS);

  // Probe AT+CREG? repeatedly until registered.
  int attempts = 0;
  while (true) {
    attempts++;
    if (DEBUG) Serial.print(F("Probing registration (AT+CREG?) attempt ")); Serial.println(attempts);
    // feed WDT while stuck in blocking startup loop
    feedWatchdog();
    if (probeGsmRegistration()) {
      if (DEBUG) Serial.println(F("GSM registered — continuing startup"));
      break;
    }

    // if module alive but not registered, give it time
    if (attempts % POWER_CYCLE_AFTER_ATTEMPTS == 0 && GSM_PWR_PIN >= 0) {
      // try soft power-cycle (if you wired a control pin)
      if (DEBUG) Serial.println(F("Power-cycle attempt crossing threshold — trying soft power-cycle"));
      softPowerCycleGsm();
    }

    if (DEBUG) Serial.println(F("Not registered yet; retrying in 5s"));
    delay(AT_RETRY_MS);
  }

  // Configure SMS mode and new message indication
  myGSM.sendAT("AT+CMGF=1", 1000);
  myGSM.sendAT("AT+CNMI=2,2,0,0,0", 1000);

  // Now we have network registration. Do heavy init now (Wire, EEPROM, attach interrupt)
  if (DEBUG) Serial.println(F("Initializing Wire/I2C and loading EEPROM records..."));
  Wire.begin();
  iEepromInitIfNeeded();
  pulsesPerLiterVar = iEepromGetPPL();

  uint16_t startLit = loadLitersFromEEPROM();
  noInterrupts(); literCounter = startLit; interrupts();
  lastSavedLiters = startLit;
  waitingForInitialLiters = (lastSequence == 0);

  // attach hall interrupt only AFTER GSM is ready and resume main operation
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), countPulse, FALLING);
  hallAttached = true;
  if (DEBUG) Serial.println(F("Hall interrupt attached"));

  // Trigger CSQ check immediately in loop
  lastGsmCheck = millis() - GSM_CHECK_INTERVAL_MS;
}

void loop() {
  // feed watchdog early in loop to prove liveness if enabled
  feedWatchdog();

  // when not ready, we still try to probe occasionally (handled earlier) — here we keep old CSQ logic too
  if (!gsmReady && millis() - lastGsmCheck >= GSM_CHECK_INTERVAL_MS) {
    lastGsmCheck = millis();
    int rssi = getGsmRssi();
    if (rssi >= 0) {
      if (DEBUG) {
        Serial.print(F("CSQ OK rssi=")); Serial.print(rssi);
        Serial.print(F(" successCount=")); Serial.println(gsmSuccessCount + 1);
      }
      if (GSM_MIN_RSSI < 0 || rssi >= GSM_MIN_RSSI) {
        gsmSuccessCount++;
        if (gsmSuccessCount >= GSM_CONSECUTIVE_SUCCESS) {
          if (gsmCandidateSince == 0) gsmCandidateSince = millis();
          unsigned long stableFor = millis() - gsmCandidateSince;
          if (DEBUG) {
            Serial.print(F("Candidate stable for ms=")); Serial.println(stableFor);
          }
          if (stableFor >= GSM_STABLE_DELAY_MS) {
            gsmReady = true;
            if (!startSent) setLED(true, false, false);
            else setLED(false, true, false);
            sendStartToTrustedNumbers();
            if (DEBUG) Serial.println(F("GSM ready; START queued (stable)"));
          } else {
            if (!startSent) { ledBlinkState = !ledBlinkState; setLED(ledBlinkState, false, false); }
            else setLED(false, true, false);
          }
        } else {
          if (!startSent) { ledBlinkState = !ledBlinkState; setLED(ledBlinkState, false, false); }
          else setLED(false, true, false);
        }
      } else {
        gsmSuccessCount = 0; gsmCandidateSince = 0;
        if (!startSent) { ledBlinkState = !ledBlinkState; setLED(ledBlinkState, false, false); }
        else setLED(false, true, false);
      }
    } else {
      gsmSuccessCount = 0; gsmCandidateSince = 0;
      if (!startSent) { ledBlinkState = !ledBlinkState; setLED(ledBlinkState, false, false); }
      else setLED(false, true, false);
    }
  }

  // normal operation after gsmReady
  if (gsmReady) {
    if (myGSM.available()) {
      if (myGSM.is_SMS()) {
        if (DEBUG) Serial.println(F("New SMS arrived"));
        String text = myGSM.SMS_Read();
        String phone = myGSM.SMS_Number();
        processIncomingSms(phone, text);
      }
    }

    serviceSmsOut();

    uint16_t cur;
    noInterrupts(); cur = literCounter; interrupts();
    static uint16_t lastSnap = 0;
    if (cur != lastSnap) {
      lastSnap = cur;
      if (DEBUG) {
        Serial.print(F("Liters: ")); Serial.println(cur);
        Serial.print(F("pulseCounter (incomplete): ")); Serial.println(pulseCounter);
      }
      if ((cur / LITERS_PER_SAVE) != (lastSavedLiters / LITERS_PER_SAVE)) {
        requestSave(cur);
      }
    }

    if (saveRequested) {
      noInterrupts();
      uint16_t litersToSave = saveRequestedLiters;
      saveRequested = false;
      interrupts();

      saveLitersToEEPROM(litersToSave);
      lastSavedLiters = litersToSave;
    }
  }

  // Always try to send queued SMS (attempt one send)
  serviceSmsOut();

  // feed watchdog at end of loop (keeps it alive if enabled)
  feedWatchdog();

  delay(100);
}
