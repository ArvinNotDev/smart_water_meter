#include <Wire.h>
#include "IranGSM.h" // this is a customized version of IranGSM library

#define RX_PIN A2
#define TX_PIN A1
#define PIN_HALL 2

#define LED_R 5
#define LED_G 6
#define LED_B 7

#define EEPROM_I2C_ADDR 0x50
#define EEPROM_TOTAL_BYTES 32768UL
#define EEPROM_PAGE_SIZE 64

#define RECORD_SIZE 8
#define RECORDS_COUNT (EEPROM_TOTAL_BYTES / RECORD_SIZE)
#define COMMIT_HI 0xA5
#define COMMIT_LO 0x5A

const bool DEBUG = true;
const String TRUSTED_NUMBER = "+989355690823";
const unsigned int PULSES_PER_LITER = 670;
const unsigned int LITERS_PER_SAVE = 5;

IranGSM myGSM(RX_PIN, TX_PIN);

volatile unsigned int pulseCounter = 0;
volatile unsigned int literCounter = 0;
unsigned int lastSavedLiters = 0;

uint32_t lastSequence = 0;
uint32_t eepromWriteIndex = 0;

#define SMS_Q_SZ 5
struct SmsJob {
  bool used = false;
  String to;
  String txt;
};
SmsJob smsQ[SMS_Q_SZ];

bool waitingForInitialLiters = true;
volatile bool saveRequested = false;
volatile uint16_t saveRequestedLiters = 0;

bool gsmReady = false;
unsigned long lastGsmCheck = 0;
bool ledBlinkState = false;

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}

void countPulse() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();
  if (now - lastMicros < 2000UL) return;
  lastMicros = now;

  pulseCounter++;
  if (pulseCounter >= PULSES_PER_LITER) {
    pulseCounter = 0;
    literCounter++;
  }
}

void eeprom_wait_ready() {
  while (true) {
    Wire.beginTransmission(EEPROM_I2C_ADDR);
    uint8_t e = Wire.endTransmission();
    if (e == 0) break;
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
      for (uint8_t i = 0; i < sub; i++) Wire.write(data[offset + i]);
      Wire.endTransmission();
      eeprom_wait_ready();
      addr += sub; offset += sub; remaining -= sub; chunk -= sub;
    }
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
    while (Wire.available() && i < chunk) out[i++] = Wire.read();
    addr += chunk; out += chunk; len -= chunk;
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
  }
  if (!any) {
    eepromWriteIndex = 0; lastSequence = 0;
    if (DEBUG) Serial.println(F("EEPROM empty; start at 0 liters"));
    return 0;
  }
  eepromWriteIndex = bestIdx + 1;
  if (eepromWriteIndex >= RECORDS_COUNT) eepromWriteIndex = 0;
  lastSequence = bestSeq;
  if (DEBUG) {
    Serial.print(F("EEPROM loaded liters=")); Serial.print(bestLit);
    Serial.print(F(" seq=")); Serial.print(bestSeq);
    Serial.print(F(" lastIdx=")); Serial.println(bestIdx);
  }
  return bestLit;
}

void saveLitersToEEPROM(uint16_t liters) {
  lastSequence++; if (lastSequence == 0) lastSequence = 1;
  eeprom_write_record(eepromWriteIndex, lastSequence, liters);
  if (DEBUG) {
    Serial.print(F("Saved liters=")); Serial.print(liters);
    Serial.print(F(" seq=")); Serial.print(lastSequence);
    Serial.print(F(" idx=")); Serial.println(eepromWriteIndex);
  }
  eepromWriteIndex++; if (eepromWriteIndex >= RECORDS_COUNT) eepromWriteIndex = 0;
}

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
      smsQ[i].used = false;
      smsQ[i].to = ""; smsQ[i].txt = "";
      if (DEBUG) Serial.println(F("SMS Sent (IranGSM)"));
      return;
    }
  }
}

void requestSave(uint16_t liters) {
  noInterrupts();
  saveRequestedLiters = liters;
  saveRequested = true;
  interrupts();
}

void processIncomingSms(const String &fromRaw, const String &bodyRaw) {
  if (DEBUG) {
    Serial.print(F("SMS FROM (raw): ")); Serial.println(fromRaw);
    Serial.print(F("BODY (raw): ")); Serial.println(bodyRaw);
  }

  String from = fromRaw;
  from.replace(" ", ""); from.replace("(", ""); from.replace(")", "");

  String trustedTail = TRUSTED_NUMBER;
  trustedTail.replace(" ", ""); trustedTail.replace("(", ""); trustedTail.replace(")", "");
  int tlen = trustedTail.length();
  int tailLen = (tlen >= 11) ? 11 : tlen;
  String trustedTailEnd = trustedTail.substring(tlen - tailLen);

  int flen = from.length();
  String fromTail = from;
  if (flen > tailLen) fromTail = from.substring(flen - tailLen);

  String body = bodyRaw;
  body.trim();
  String bodyUpper = body;
  bodyUpper.toUpperCase();

  if (fromTail == trustedTailEnd) {
    if (waitingForInitialLiters) {
      bool allDigits = true;
      String tmp = body; tmp.trim();
      if (tmp.length() == 0) allDigits = false;
      for (uint16_t i = 0; i < tmp.length() && allDigits; i++) {
        if (!isDigit(tmp[i])) { allDigits = false; break; }
      }
      if (allDigits) {
        uint16_t v = (uint16_t) tmp.toInt();
        noInterrupts(); literCounter = v; interrupts();
        waitingForInitialLiters = false;
        enqueueSms(from, "LITER SET TO " + String(v));
        requestSave(v);
        return;
      }
    }

    if (bodyUpper == "GET") {
      uint16_t cur; noInterrupts(); cur = literCounter; interrupts();
      enqueueSms(from, "LITERS=" + String(cur));
      return;
    }

    if (bodyUpper == "RESET") {
      noInterrupts(); literCounter = 0; interrupts();
      enqueueSms(from, "LITERS RESET");
      requestSave(0);
      return;
    }

    if (bodyUpper.startsWith("POST ")) {
      String num = body.substring(5);
      num.trim();
      bool allDigits = true;
      if (num.length() == 0) allDigits = false;
      for (uint16_t i = 0; i < num.length() && allDigits; i++) {
        if (!isDigit(num[i])) { allDigits = false; break; }
      }
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

    if (bodyUpper == "I SENT" || bodyUpper == "SENT" || body.indexOf("فرست") != -1 || body.indexOf("فرستادم") != -1) {
      enqueueSms(from, "SAVED_LITERS=" + String(lastSavedLiters));
      return;
    }

    enqueueSms(from, "INVALID COMMAND");
  } else {
    if (DEBUG) Serial.println(F("Ignoring unauthorized sender"));
  }
}

bool checkGsmConnected() {
  String resp = myGSM.sendAT("AT+CSQ", 2000);

  int idx = resp.indexOf("+CSQ:");
  if (idx == -1) return false;

  int comma = resp.indexOf(',', idx);
  if (comma == -1) return false;

  int rssi = resp.substring(idx + 6, comma).toInt();

  if (rssi >= 0 && rssi <= 31) return true;
  return false;
}


void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(2000);

  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), countPulse, FALLING);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLED(false, false, false);

  delay(1500);

  if (DEBUG) Serial.println(F("=== Water meter starting (IranGSM mode) ==="));

  uint16_t startLit = loadLitersFromEEPROM();
  noInterrupts(); literCounter = startLit; interrupts();
  lastSavedLiters = startLit;

  if (lastSequence != 0) waitingForInitialLiters = false;
  else waitingForInitialLiters = true;
}

void loop() {
  if (!gsmReady && millis() - lastGsmCheck >= 2000) {
    lastGsmCheck = millis();
    bool connected = checkGsmConnected();

    if (connected) {
      gsmReady = true;
      setLED(false, true, false);
      enqueueSms(TRUSTED_NUMBER, "START");
      if (DEBUG) Serial.println(F("GSM connected, START sent"));
    } else {
      ledBlinkState = !ledBlinkState;
      setLED(ledBlinkState, false, false);
      if (DEBUG) Serial.println(F("GSM not connected, blinking red"));
    }
  }

  if (gsmReady) {
    if (myGSM.available()) {
      if (myGSM.is_SMS()) {
        if (DEBUG) Serial.println(F("=====>   new SMS arrived (IranGSM)"));
        String SMS_Text = myGSM.SMS_Read();
        String Phone_Number = myGSM.SMS_Number();
        processIncomingSms(Phone_Number, SMS_Text);
      }
    }

    serviceSmsOut();

    uint16_t cur;
    noInterrupts(); cur = literCounter; interrupts();
    static uint16_t lastSnap = 0;
    if (cur != lastSnap) {
      lastSnap = cur;
      if (DEBUG) { Serial.print(F("Liters: ")); Serial.println(cur); }
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

  delay(100);
}
