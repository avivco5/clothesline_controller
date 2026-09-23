#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

/*
 * Outdoor rotating clothesline motor safety controller
 * NodeMCU v1.0 (ESP8266) + Pololu VNH5019 Motor Driver Carrier (#1451)
 *
 * Wiring (per project spec):
 *   Motor supply (+) -> VNH5019 VIN   (10A fuse in this line)
 *   Motor supply (-) -> VNH5019 GND
 *   Motor            -> OUTA / OUTB
 *   NodeMCU 3V3      -> VNH5019 VDD
 *   NodeMCU GND      -> VNH5019 logic GND + motor supply GND
 *   VNH5019 PWM      -> D5  / GPIO14
 *   VNH5019 INA      -> D6  / GPIO12
 *   VNH5019 INB      -> D7  / GPIO13
 *   VNH5019 ENA/DIAGA-> D1  / GPIO5
 *   VNH5019 ENB/DIAGB-> D2  / GPIO4
 *   VNH5019 CS       -> A0
 *   VNH5019 CS_DIS   -> not connected
 *   BabyBuck 5V out  -> NodeMCU VIN/5V pin (never into 3V3)
 *
 * FLAGGED — please verify on the bench, do not assume:
 *   1) ENA/DIAGA and ENB/DIAGB are combined enable+fault pins on the VNH5019.
 *      The driver only enables a half-bridge while its pin is pulled HIGH,
 *      and pulls it LOW itself on a fault. This code uses the ESP8266's
 *      internal weak pull-up (INPUT_PULLUP) to hold the pin high by default.
 *      CONFIRM the motor actually spins and that the pin reads HIGH with a
 *      multimeter with no fault present — if your carrier board revision
 *      has no onboard pull-up and the ESP8266 internal pull-up is too weak
 *      in your setup, add an external 10k pull-up to 3V3 on both lines.
 *   2) NodeMCU A0 reads 0-3.3V via an onboard resistor divider external to
 *      the ESP8266 chip. This assumes a standard NodeMCU v1.0 divider.
 *      Confirm against your exact board revision.
 *   3) ESP8266 analogWrite() is a software PWM. Very high frequency
 *      combined with high duty resolution is a known soft limit of the
 *      Arduino core. PWM_RESOLUTION below is a conservative default —
 *      verify with a scope that the PWM and CS signals are clean, and
 *      reduce PWM_RESOLUTION and/or PWM_FREQUENCY_HZ further if not.
 *   4) CS_DIS is left unconnected per spec; this assumes the carrier
 *      board's default bias keeps the CS output active. Confirm against
 *      your board's schematic if current readings look wrong.
 *
 * Runtime-tunable protection settings (overload threshold, timing, recovery
 * speed/attempts) can be changed from the web UI's "הגדרות מתקדמות" panel.
 * These live in RAM ONLY — by design (see /api/settings) they always reset
 * to the safe DEFAULT_* values below on every boot/power loss, so a bad
 * value entered during testing can never persist into an unattended run.
 *
 * AUTO-START ON BOOT — explicit deviation from the original "motor must
 * never start automatically" requirement, requested by the project owner.
 * At every boot the unit waits AUTOSTART_DELAY_MS, then starts driving
 * FORWARD at AUTOSTART_SPEED_PERCENT — UNLESS a VNH5019 diagnostic fault
 * is present, in which case it enters LOCKOUT instead and never autostarts.
 * Any manual command (stop/forward/reverse/unlock) received before the
 * delay expires cancels the pending autostart.
 * FLAGGED: this does NOT protect against a physical brownout/reset caused
 * by the stall current itself sagging the supply — if that happens, the
 * DIAG pins may read clean at boot (the motor already stopped when power
 * dropped), so the unit will autostart forward again into a snag that
 * hasn't been cleared. Set AUTOSTART_ENABLED to false below to fall back
 * to the original always-stopped-at-boot behavior if this is a concern.
 *
 * AUTO-RETRY FROM LOCKOUT — second explicit deviation from "LOCKOUT
 * requires the user to press Unlock", also requested by the project
 * owner. This applies ONLY to a LOCKOUT caused by overload that never
 * cleared (2 failed recovery attempts, or overload persisting through a
 * recovery reverse) — every autoRetryMs (see below), the unit clears the
 * attempt counter and resumes the previously requested direction on its
 * own, on the theory that a snag may free itself over time with no one
 * around to press Unlock.
 * A LOCKOUT caused by a VNH5019 diagnostic fault (DIAGA/DIAGB) is
 * deliberately EXCLUDED from auto-retry — that can indicate a real
 * driver/motor electrical fault, not just a stuck rope, and still
 * requires a human to press Unlock.
 *
 * WI-FI AUTO-OFF — the AP and web UI (see WIFI_ON_DURATION_MS below) are
 * for initial setup/tuning only, per the project owner: once the right
 * values were found, the unit is meant to just run from the burned-in
 * defaults with no network active. Wi-Fi/the server shut down
 * WIFI_ON_DURATION_MS after boot and never come back until the next
 * reset/reflash — there is no way to reach the web UI after that window
 * without power-cycling the board. The motor/protection state machine
 * does not depend on Wi-Fi at all, so this has no effect on it.
 */

// ── Wi-Fi ────────────────────────────────────────────────────────────────
const char* AP_SSID = "Clothesline-Control";
const char* AP_PASSWORD = "clothesline";

// The AP + web UI are for initial setup/tuning only, not normal operation.
// They shut off this many ms after boot; from then on the unit runs purely
// from the burned-in defaults above, with no network active at all.
const uint32_t WIFI_ON_DURATION_MS = 300000; // 5 minutes

// ── Pins ─────────────────────────────────────────────────────────────────
const uint8_t PIN_PWM = D5;          // GPIO14
const uint8_t PIN_INA = D6;          // GPIO12
const uint8_t PIN_INB = D7;          // GPIO13
const uint8_t PIN_ENA_DIAGA = D1;    // GPIO5
const uint8_t PIN_ENB_DIAGB = D2;    // GPIO4
const uint8_t PIN_CURRENT = A0;

// ── Motor direction polarity ─────────────────────────────────────────────
const bool FORWARD_INA_HIGH = true;

// ── PWM configuration (see FLAGGED note #3 above) ───────────────────────
const uint16_t PWM_RESOLUTION = 200;   // analogWriteRange() steps
const uint32_t PWM_FREQUENCY_HZ = 10000;

// ── Current sensing (VNH5019 CS: ~0.14 V/A, valid only while driving) ───
const float ADC_MAX_COUNTS = 1023.0f;      // ESP8266 ADC resolution
const float ADC_FULL_SCALE_V = 3.3f;       // NodeMCU A0 divider (see note #2)
const float CURRENT_SENSE_V_PER_A = 0.140f;
const uint8_t CURRENT_OVERSAMPLES = 4;

const uint32_t SAMPLE_INTERVAL_MS = 20;
const uint32_t STARTUP_SETTLE_MS = 300; // ignore DIAG pins briefly after boot

// ── Auto-start on boot (see FLAGGED note near top of file) ─────────────
const bool AUTOSTART_ENABLED = true;
const uint32_t AUTOSTART_DELAY_MS = 4000; // grace period before driving off
const uint8_t AUTOSTART_SPEED_PERCENT = 100;

// ── Runtime-tunable protection settings ─────────────────────────────────
// Defaults + safe clamp ranges. Actual live values are the non-const
// globals further down (overcurrentA, overcurrentDelayMs, ...), editable
// from the web UI and always reset to these defaults on boot.
const float DEFAULT_OVERCURRENT_A = 3.5f;
const float OVERCURRENT_A_MIN = 1.5f;
const float OVERCURRENT_A_MAX = 9.0f;      // stay under the 10A fuse rating

const uint32_t DEFAULT_OVERCURRENT_DELAY_MS = 2000;
const uint32_t OVERCURRENT_DELAY_MS_MIN = 200;
const uint32_t OVERCURRENT_DELAY_MS_MAX = 10000;

const uint32_t DEFAULT_REVERSE_TIME_MS = 15000;
const uint32_t REVERSE_TIME_MS_MIN = 500;
const uint32_t REVERSE_TIME_MS_MAX = 15000;

const uint32_t DEFAULT_STOP_DELAY_MS = 4000;
const uint32_t STOP_DELAY_MS_MIN = 100;
const uint32_t STOP_DELAY_MS_MAX = 5000;

const uint8_t DEFAULT_MAX_RECOVERY_ATTEMPTS = 2;
const uint8_t MAX_RECOVERY_ATTEMPTS_MIN = 1;
const uint8_t MAX_RECOVERY_ATTEMPTS_MAX = 5;

const uint32_t DEFAULT_RECOVERY_RESET_MS = 60000;
const uint32_t RECOVERY_RESET_MS_MIN = 5000;
const uint32_t RECOVERY_RESET_MS_MAX = 600000;

// Auto-retry from an overload-caused LOCKOUT only (never from a driver
// fault LOCKOUT) — see note near top of file.
const uint32_t DEFAULT_AUTO_RETRY_MS = 1800000; // 30 minutes
const uint32_t AUTO_RETRY_MS_MIN = 30000;      // 30 seconds
const uint32_t AUTO_RETRY_MS_MAX = 1800000;    // 30 minutes

ESP8266WebServer server(80);

enum MotorState {
  IDLE,
  DRIVE,
  STOP_BEFORE_MANUAL,
  STOP_BEFORE_REVERSE,
  RECOVERY_REVERSE,
  STOP_AFTER_RECOVERY,
  LOCKOUT
};

MotorState state = IDLE;

bool requestedRun = false;
bool requestedForward = true;
bool activeForward = true;

const uint16_t requestedPwm = PWM_RESOLUTION; // speed is fixed at 100%, no user control
uint8_t recoveryAttempts = 0;

float filteredCurrentA = 0.0f;

uint32_t stateStartMs = 0;
uint32_t overloadStartMs = 0;
uint32_t healthyForwardStartMs = 0;
uint32_t lastSampleMs = 0;

bool autoStartPending = AUTOSTART_ENABLED;
bool wifiActive = true;

// Live protection settings — RAM only, reset to DEFAULT_* on every boot.
float overcurrentA = DEFAULT_OVERCURRENT_A;
uint32_t overcurrentDelayMs = DEFAULT_OVERCURRENT_DELAY_MS;
uint32_t reverseTimeMs = DEFAULT_REVERSE_TIME_MS;
const uint16_t recoveryPwm = PWM_RESOLUTION; // recovery speed is fixed at 100%, no user control
uint32_t stopDelayMs = DEFAULT_STOP_DELAY_MS;
uint8_t maxRecoveryAttempts = DEFAULT_MAX_RECOVERY_ATTEMPTS;
uint32_t recoveryResetMs = DEFAULT_RECOVERY_RESET_MS;
uint32_t autoRetryMs = DEFAULT_AUTO_RETRY_MS;

bool lockoutIsFault = false;
uint32_t lockoutStartMs = 0;
bool autoRetryArmed = false;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="he" dir="rtl">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>בקר מתלה</title>
<style>
body { font-family: Arial, sans-serif; background: #f2f4f7; margin: 0; padding: 20px; }
.card { max-width: 440px; margin: auto; background: white; padding: 22px; border-radius: 16px; box-shadow: 0 3px 16px #0002; }
h1 { margin-top: 0; font-size: 25px; }
.status { padding: 14px; background: #edf4ff; border-radius: 10px; line-height: 1.8; margin-bottom: 20px; }
button { width: 100%; border: 0; padding: 16px; margin: 6px 0; border-radius: 10px; font-size: 18px; font-weight: bold; color: white; }
.forward { background: #16803c; }
.reverse { background: #2563eb; }
.stop { background: #d92525; }
.unlock { background: #c07800; }
input { width: 100%; }
.value { font-weight: bold; font-size: 20px; }
details { margin-top: 18px; }
summary { cursor: pointer; font-weight: bold; padding: 10px 0; }
.field { margin-bottom: 12px; }
.field label { display: block; font-size: 13px; color: #444; margin-bottom: 4px; }
.field input { width: 100%; box-sizing: border-box; padding: 8px; border: 1px solid #ccc; border-radius: 6px; font-size: 15px; }
.settingsBtns { display: flex; gap: 8px; margin-top: 10px; }
.settingsBtns button { margin: 0; }
.save { background: #1f8a3b; }
.reset { background: #6b7280; }
.note { font-size: 12px; color: #8a6d00; background: #fff7db; border-radius: 8px; padding: 8px 10px; margin-bottom: 12px; }
</style>
</head>
<body>
<div class="card">
<h1>בקר מתלה חבלים</h1>
<div class="status">
מצב: <b id="state">טוען...</b><br>
זרם: <b id="current">-</b> A<br>
כיוון מבוקש: <b id="direction">-</b><br>
ניסיונות שחרור: <b id="attempts">-</b>
</div>

<div class="note">המנוע פועל תמיד במהירות מלאה (100%) — אין אפשרות להגדיר מהירות נמוכה יותר.</div>

<button class="forward" onclick="sendCommand('forward')">קדימה</button>
<button class="reverse" onclick="sendCommand('reverse')">אחורה</button>
<button class="stop" onclick="sendCommand('stop')">עצור</button>
<button class="unlock" onclick="sendCommand('unlock')">שחרר נעילה</button>

<details>
<summary>הגדרות מתקדמות</summary>
<div class="note">ההגדרות נשמרות בזיכרון בלבד ויחזרו לברירת המחדל הבטוחה בכל אתחול/ניתוק חשמל.</div>

<div class="field">
<label for="setOvercurrentA">סף זרם עומס יתר (אמפר)</label>
<input id="setOvercurrentA" type="number" step="0.1">
</div>
<div class="field">
<label for="setOverDelay">זמן עד טריגר עומס (שניות)</label>
<input id="setOverDelay" type="number" step="0.1">
</div>
<div class="field">
<label for="setReverseTime">משך היפוך שחרור (שניות)</label>
<input id="setReverseTime" type="number" step="0.5">
</div>
<div class="field">
<label for="setStopDelay">זמן עצירה בין החלפת כיוון (מילישניות)</label>
<input id="setStopDelay" type="number" step="50">
</div>
<div class="field">
<label for="setMaxAttempts">מספר ניסיונות שחרור מותרים</label>
<input id="setMaxAttempts" type="number" step="1">
</div>
<div class="field">
<label for="setResetTime">זמן ריצה תקינה לאיפוס מונה (שניות)</label>
<input id="setResetTime" type="number" step="5">
</div>
<div class="field">
<label for="setAutoRetry">ניסיון חוזר אוטומטי מנעילת עומס יתר (שניות)</label>
<input id="setAutoRetry" type="number" step="10">
</div>

<div class="settingsBtns">
<button class="save" onclick="saveSettings()">שמור הגדרות</button>
<button class="reset" onclick="resetSettings()">אפס לברירת מחדל</button>
</div>
</details>
</div>

<script>
function sendCommand(command) {
  fetch('/api/set?cmd=' + command)
    .then(updateStatus);
}

function updateStatus() {
  fetch('/api/status')
    .then(response => response.json())
    .then(data => {
      document.getElementById('state').innerText = data.state;
      document.getElementById('current').innerText = data.current;
      document.getElementById('direction').innerText = data.direction;
      document.getElementById('attempts').innerText = data.attempts;
    });
}

function setField(id, val, min, max) {
  const el = document.getElementById(id);
  if (!el) return;
  el.min = min;
  el.max = max;
  el.value = val;
}

function applySettingsToForm(s) {
  setField('setOvercurrentA', s.overcurrentA, s.overcurrentA_min, s.overcurrentA_max);
  setField('setOverDelay', s.overcurrentDelayS, s.overcurrentDelayS_min, s.overcurrentDelayS_max);
  setField('setReverseTime', s.reverseTimeS, s.reverseTimeS_min, s.reverseTimeS_max);
  setField('setStopDelay', s.stopDelayMs, s.stopDelayMs_min, s.stopDelayMs_max);
  setField('setMaxAttempts', s.maxAttempts, s.maxAttempts_min, s.maxAttempts_max);
  setField('setResetTime', s.resetTimeS, s.resetTimeS_min, s.resetTimeS_max);
  setField('setAutoRetry', s.autoRetryS, s.autoRetryS_min, s.autoRetryS_max);
}

function loadSettings() {
  fetch('/api/settings')
    .then(r => r.json())
    .then(applySettingsToForm);
}

function saveSettings() {
  const params = new URLSearchParams({
    overcurrentA: document.getElementById('setOvercurrentA').value,
    overcurrentDelayS: document.getElementById('setOverDelay').value,
    reverseTimeS: document.getElementById('setReverseTime').value,
    stopDelayMs: document.getElementById('setStopDelay').value,
    maxAttempts: document.getElementById('setMaxAttempts').value,
    resetTimeS: document.getElementById('setResetTime').value,
    autoRetryS: document.getElementById('setAutoRetry').value
  });
  fetch('/api/settings/set?' + params.toString())
    .then(r => r.json())
    .then(applySettingsToForm);
}

function resetSettings() {
  fetch('/api/settings/reset')
    .then(r => r.json())
    .then(applySettingsToForm);
}

setInterval(updateStatus, 800);
updateStatus();
loadSettings();
</script>
</body>
</html>
)rawliteral";

float readCurrentA() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < CURRENT_OVERSAMPLES; i++) {
    sum += analogRead(PIN_CURRENT);
  }
  float raw = (float)sum / CURRENT_OVERSAMPLES;
  float voltage = (raw * ADC_FULL_SCALE_V) / ADC_MAX_COUNTS;
  return voltage / CURRENT_SENSE_V_PER_A;
}

bool driverFault() {
  return digitalRead(PIN_ENA_DIAGA) == LOW ||
         digitalRead(PIN_ENB_DIAGB) == LOW;
}

void stopMotor() {
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_INA, LOW);
  digitalWrite(PIN_INB, LOW);
}

void driveMotor(bool forward, uint16_t pwmValue) {
  if (pwmValue == 0) {
    stopMotor();
    return;
  }

  analogWrite(PIN_PWM, 0);

  bool inaHigh = forward ? FORWARD_INA_HIGH : !FORWARD_INA_HIGH;

  digitalWrite(PIN_INA, inaHigh ? HIGH : LOW);
  digitalWrite(PIN_INB, inaHigh ? LOW : HIGH);

  analogWrite(PIN_PWM, pwmValue);
}

void enterLockout(bool isFault) {
  stopMotor();
  requestedRun = false;
  filteredCurrentA = 0.0f;
  state = LOCKOUT;
  lockoutIsFault = isFault;
  lockoutStartMs = millis();
  autoRetryArmed = !isFault;
  Serial.println(isFault ? "LOCKOUT (driver fault)" : "LOCKOUT (overload not cleared)");
}

void startRequestedDrive() {
  if (!requestedRun || requestedPwm == 0) {
    stopMotor();
    state = IDLE;
    return;
  }

  activeForward = requestedForward;
  filteredCurrentA = 0.0f;
  overloadStartMs = 0;
  healthyForwardStartMs = millis();

  driveMotor(activeForward, requestedPwm);
  state = DRIVE;
}

void scheduleManualStart() {
  stopMotor();
  overloadStartMs = 0;
  state = STOP_BEFORE_MANUAL;
  stateStartMs = millis();
}

bool isDriving() {
  return state == DRIVE || state == RECOVERY_REVERSE;
}

void updateCurrentAndProtection(uint32_t now) {
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;
  }

  lastSampleMs = now;

  float currentA = readCurrentA();
  filteredCurrentA = 0.8f * filteredCurrentA + 0.2f * currentA;

  if (filteredCurrentA >= overcurrentA) {
    if (overloadStartMs == 0) {
      overloadStartMs = now;
      Serial.println("Overcurrent timer started");
    }

    if (now - overloadStartMs >= overcurrentDelayMs) {
      overloadStartMs = 0;

      if (state == DRIVE &&
          recoveryAttempts < maxRecoveryAttempts) {
        stopMotor();
        state = STOP_BEFORE_REVERSE;
        stateStartMs = now;
        Serial.println("Recovery sequence started");
      } else {
        Serial.println("Overload persisted / recovery attempts exhausted");
        enterLockout(false);
      }
    }
  } else {
    overloadStartMs = 0;
  }
}

const char* stateText() {
  switch (state) {
    case IDLE: return "עצור";
    case DRIVE: return "פועל";
    case STOP_BEFORE_MANUAL: return "מחליף כיוון";
    case STOP_BEFORE_REVERSE: return "לפני שחרור";
    case RECOVERY_REVERSE: return "שחרור אוטומטי";
    case STOP_AFTER_RECOVERY: return "חוזר לכיוון המקורי";
    case LOCKOUT:
      return lockoutIsFault ? "נעול - תקלת חומרה, נדרש שחרור ידני"
                             : "נעול - עומס יתר, ינסה שוב אוטומטית";
  }

  return "לא ידוע";
}

void buildStatusJson(char* buf, size_t bufSize) {
  snprintf(buf, bufSize,
    "{\"state\":\"%s\",\"current\":%.2f,\"direction\":\"%s\",\"attempts\":%u}",
    stateText(),
    filteredCurrentA,
    requestedForward ? "קדימה" : "אחורה",
    (unsigned)recoveryAttempts
  );
}

void setOvercurrentA(float v) {
  overcurrentA = constrain(v, OVERCURRENT_A_MIN, OVERCURRENT_A_MAX);
}

void setOvercurrentDelayMs(uint32_t v) {
  overcurrentDelayMs = constrain(v, OVERCURRENT_DELAY_MS_MIN, OVERCURRENT_DELAY_MS_MAX);
}

void setReverseTimeMs(uint32_t v) {
  reverseTimeMs = constrain(v, REVERSE_TIME_MS_MIN, REVERSE_TIME_MS_MAX);
}

void setStopDelayMs(uint32_t v) {
  stopDelayMs = constrain(v, STOP_DELAY_MS_MIN, STOP_DELAY_MS_MAX);
}

void setMaxRecoveryAttempts(uint8_t v) {
  maxRecoveryAttempts = constrain(v, MAX_RECOVERY_ATTEMPTS_MIN, MAX_RECOVERY_ATTEMPTS_MAX);
}

void setRecoveryResetMs(uint32_t v) {
  recoveryResetMs = constrain(v, RECOVERY_RESET_MS_MIN, RECOVERY_RESET_MS_MAX);
}

void setAutoRetryMs(uint32_t v) {
  autoRetryMs = constrain(v, AUTO_RETRY_MS_MIN, AUTO_RETRY_MS_MAX);
}

void resetSettingsToDefaults() {
  overcurrentA = DEFAULT_OVERCURRENT_A;
  overcurrentDelayMs = DEFAULT_OVERCURRENT_DELAY_MS;
  reverseTimeMs = DEFAULT_REVERSE_TIME_MS;
  stopDelayMs = DEFAULT_STOP_DELAY_MS;
  maxRecoveryAttempts = DEFAULT_MAX_RECOVERY_ATTEMPTS;
  recoveryResetMs = DEFAULT_RECOVERY_RESET_MS;
  autoRetryMs = DEFAULT_AUTO_RETRY_MS;
  Serial.println("Protection settings reset to defaults");
}

void buildSettingsJson(char* buf, size_t bufSize) {
  snprintf(buf, bufSize,
    "{"
    "\"overcurrentA\":%.2f,\"overcurrentA_min\":%.2f,\"overcurrentA_max\":%.2f,"
    "\"overcurrentDelayS\":%.2f,\"overcurrentDelayS_min\":%.2f,\"overcurrentDelayS_max\":%.2f,"
    "\"reverseTimeS\":%.2f,\"reverseTimeS_min\":%.2f,\"reverseTimeS_max\":%.2f,"
    "\"stopDelayMs\":%u,\"stopDelayMs_min\":%u,\"stopDelayMs_max\":%u,"
    "\"maxAttempts\":%u,\"maxAttempts_min\":%u,\"maxAttempts_max\":%u,"
    "\"resetTimeS\":%.1f,\"resetTimeS_min\":%.1f,\"resetTimeS_max\":%.1f,"
    "\"autoRetryS\":%.0f,\"autoRetryS_min\":%.0f,\"autoRetryS_max\":%.0f"
    "}",
    overcurrentA, OVERCURRENT_A_MIN, OVERCURRENT_A_MAX,
    overcurrentDelayMs / 1000.0f, OVERCURRENT_DELAY_MS_MIN / 1000.0f, OVERCURRENT_DELAY_MS_MAX / 1000.0f,
    reverseTimeMs / 1000.0f, REVERSE_TIME_MS_MIN / 1000.0f, REVERSE_TIME_MS_MAX / 1000.0f,
    (unsigned)stopDelayMs, (unsigned)STOP_DELAY_MS_MIN, (unsigned)STOP_DELAY_MS_MAX,
    (unsigned)maxRecoveryAttempts, (unsigned)MAX_RECOVERY_ATTEMPTS_MIN, (unsigned)MAX_RECOVERY_ATTEMPTS_MAX,
    recoveryResetMs / 1000.0f, RECOVERY_RESET_MS_MIN / 1000.0f, RECOVERY_RESET_MS_MAX / 1000.0f,
    autoRetryMs / 1000.0f, AUTO_RETRY_MS_MIN / 1000.0f, AUTO_RETRY_MS_MAX / 1000.0f
  );
}

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  char json[192];
  buildStatusJson(json, sizeof(json));
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleSettingsGet() {
  char json[640];
  buildSettingsJson(json, sizeof(json));
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleSettingsSet() {
  if (server.hasArg("overcurrentA")) {
    setOvercurrentA(server.arg("overcurrentA").toFloat());
  }
  if (server.hasArg("overcurrentDelayS")) {
    setOvercurrentDelayMs((uint32_t)(server.arg("overcurrentDelayS").toFloat() * 1000.0f));
  }
  if (server.hasArg("reverseTimeS")) {
    setReverseTimeMs((uint32_t)(server.arg("reverseTimeS").toFloat() * 1000.0f));
  }
  if (server.hasArg("stopDelayMs")) {
    setStopDelayMs((uint32_t)server.arg("stopDelayMs").toInt());
  }
  if (server.hasArg("maxAttempts")) {
    setMaxRecoveryAttempts((uint8_t)server.arg("maxAttempts").toInt());
  }
  if (server.hasArg("resetTimeS")) {
    setRecoveryResetMs((uint32_t)(server.arg("resetTimeS").toFloat() * 1000.0f));
  }
  if (server.hasArg("autoRetryS")) {
    setAutoRetryMs((uint32_t)(server.arg("autoRetryS").toFloat() * 1000.0f));
  }

  Serial.printf(
    "Settings updated: overcurrentA=%.2f delay=%lums reverse=%lums stopDelay=%lums maxAttempts=%u resetTime=%lums autoRetry=%lums\n",
    overcurrentA, (unsigned long)overcurrentDelayMs, (unsigned long)reverseTimeMs,
    (unsigned long)stopDelayMs,
    (unsigned)maxRecoveryAttempts, (unsigned long)recoveryResetMs, (unsigned long)autoRetryMs
  );

  char json[640];
  buildSettingsJson(json, sizeof(json));
  server.send(200, "application/json", json);
}

void handleSettingsReset() {
  resetSettingsToDefaults();
  char json[640];
  buildSettingsJson(json, sizeof(json));
  server.send(200, "application/json", json);
}

void handleSet() {
  autoStartPending = false; // any manual command cancels a pending autostart

  // Speed is fixed at 100% (PWM_RESOLUTION) — there is no user-facing speed
  // control. Lower speeds did not produce enough torque to move the motor,
  // so the option to run below full speed was removed entirely.

  String command = server.arg("cmd");

  if (command == "stop") {
    requestedRun = false;
    stopMotor();
    overloadStartMs = 0;
    // A manual stop must never clear a LOCKOUT — only "unlock" may do that.
    // It does cancel a pending auto-retry: an explicit Stop means "wait
    // for me", not "keep trying on your own".
    autoRetryArmed = false;
    if (state != LOCKOUT) {
      state = IDLE;
    }
  }

  if (command == "unlock") {
    if (state == LOCKOUT) {
      recoveryAttempts = 0;
      requestedRun = false;
      stopMotor();
      state = IDLE;
      Serial.println("Unlocked by user");
    }
  }

  if (command == "forward" || command == "reverse") {
    if (state != LOCKOUT) {
      bool newForward = (command == "forward");
      requestedForward = newForward;
      requestedRun = requestedPwm > 0;

      if (state == DRIVE && newForward == activeForward) {
        // Same direction already running: just update speed, no need to
        // go through the stop/wait/reverse sequence (that is reserved for
        // an actual direction change, per spec item 4).
        if (requestedRun) {
          driveMotor(activeForward, requestedPwm);
        } else {
          stopMotor();
          state = IDLE;
        }
      } else {
        scheduleManualStart();
      }
    }
  }

  char json[192];
  buildStatusJson(json, sizeof(json));
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_INA, OUTPUT);
  pinMode(PIN_INB, OUTPUT);

  // ENA/DIAGA and ENB/DIAGB are combined enable+fault pins on the VNH5019.
  // INPUT_PULLUP holds them high (enabling the driver) by default; the
  // driver itself pulls a pin low to report a fault. See FLAGGED note #1
  // at the top of this file — verify this assumption on the bench.
  pinMode(PIN_ENA_DIAGA, INPUT_PULLUP);
  pinMode(PIN_ENB_DIAGB, INPUT_PULLUP);

  analogWriteRange(PWM_RESOLUTION);
  analogWriteFreq(PWM_FREQUENCY_HZ);

  stopMotor();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(
    IPAddress(192, 168, 4, 1),
    IPAddress(192, 168, 4, 1),
    IPAddress(255, 255, 255, 0)
  );
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/set", HTTP_GET, handleSet);
  server.on("/api/settings", HTTP_GET, handleSettingsGet);
  server.on("/api/settings/set", HTTP_GET, handleSettingsSet);
  server.on("/api/settings/reset", HTTP_GET, handleSettingsReset);
  server.begin();

  Serial.println();
  Serial.println("WiFi AP started");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.println("Open: http://192.168.4.1");
  Serial.print("DIAGA initial (expect HIGH, no fault): ");
  Serial.println(digitalRead(PIN_ENA_DIAGA) == HIGH ? "HIGH" : "LOW");
  Serial.print("DIAGB initial (expect HIGH, no fault): ");
  Serial.println(digitalRead(PIN_ENB_DIAGB) == HIGH ? "HIGH" : "LOW");

  if (AUTOSTART_ENABLED) {
    Serial.printf("Autostart armed: forward at %u%% in %lu ms unless a fault is detected\n",
      (unsigned)AUTOSTART_SPEED_PERCENT, (unsigned long)AUTOSTART_DELAY_MS);
  }
  Serial.printf("Wi-Fi/web UI will turn off after %lu ms; use that window to tune settings\n",
    (unsigned long)WIFI_ON_DURATION_MS);
}

void loop() {
  uint32_t now = millis();

  if (wifiActive && now >= WIFI_ON_DURATION_MS) {
    wifiActive = false;
    server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("Wi-Fi/web UI turned off — running autonomously from burned-in defaults");
  }

  if (wifiActive) {
    server.handleClient();
  }

  // Diagnostic fault check runs every cycle, regardless of motor state,
  // so a fault occurring while idle is not missed until the next drive.
  // A short settle window avoids a false trip while pins stabilize at boot.
  if (state != LOCKOUT && now > STARTUP_SETTLE_MS && driverFault()) {
    Serial.println("VNH5019 diagnostic fault detected");
    enterLockout(true);
  }

  if (autoStartPending) {
    if (state == LOCKOUT) {
      // A fault was present at boot — never autostart into it.
      autoStartPending = false;
      Serial.println("Autostart cancelled: fault present at boot");
    } else if (state == IDLE && now >= AUTOSTART_DELAY_MS) {
      autoStartPending = false;
      requestedForward = true;
      requestedRun = true; // requestedPwm is fixed at PWM_RESOLUTION (100%)
      Serial.println("Autostart: no fault detected, starting forward motion");
      startRequestedDrive();
    }
  }

  if (state == LOCKOUT && autoRetryArmed && now - lockoutStartMs >= autoRetryMs) {
    autoRetryArmed = false;
    Serial.println("Auto-retry: clearing overload lockout, resuming requested direction");
    recoveryAttempts = 0;
    requestedRun = requestedPwm > 0;
    startRequestedDrive();
  }

  if (state == LOCKOUT || state == IDLE) {
    filteredCurrentA = 0.0f;
    return;
  }

  if (isDriving()) {
    updateCurrentAndProtection(now);
  }

  switch (state) {
    case STOP_BEFORE_MANUAL:
      if (now - stateStartMs >= stopDelayMs) {
        startRequestedDrive();
      }
      break;

    case STOP_BEFORE_REVERSE:
      if (now - stateStartMs >= stopDelayMs) {
        recoveryAttempts++;
        filteredCurrentA = 0.0f;
        overloadStartMs = 0;

        driveMotor(!activeForward, recoveryPwm);
        state = RECOVERY_REVERSE;
        stateStartMs = now;

        Serial.print("Recovery reverse started, attempt ");
        Serial.println(recoveryAttempts);
      }
      break;

    case RECOVERY_REVERSE:
      if (now - stateStartMs >= reverseTimeMs) {
        stopMotor();
        state = STOP_AFTER_RECOVERY;
        stateStartMs = now;

        Serial.println("Recovery reverse finished");
      }
      break;

    case STOP_AFTER_RECOVERY:
      if (now - stateStartMs >= stopDelayMs) {
        startRequestedDrive();
      }
      break;

    case DRIVE:
      if (filteredCurrentA < overcurrentA &&
          now - healthyForwardStartMs >= recoveryResetMs) {
        recoveryAttempts = 0;
      }
      break;

    default:
      break;
  }

  static uint32_t lastDiagLogMs = 0;
  if (isDriving() && now - lastDiagLogMs >= 1000) {
    lastDiagLogMs = now;
    Serial.printf("state=%s current=%.2fA speed=%u/%u dir=%s attempts=%u\n",
      stateText(), filteredCurrentA, requestedPwm, PWM_RESOLUTION,
      activeForward ? "fwd" : "rev", (unsigned)recoveryAttempts);
  }
}
