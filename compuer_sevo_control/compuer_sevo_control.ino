/*
 * compuer_sevo_control.ino — PCA9685 dual-arm driver (ESP32)
 *
 * Protocol: "L_MG,L_SG,R_MG,R_SG\n"  e.g. "0,92,180,92\n"
 *
 * MG996R (ch 0, 2): continuous EMA ramp + PWM updates (unchanged behavior).
 * SG90   (ch 1, 3): target deadband, stepped interpolation, auto PWM-off sleep.
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ── PCA9685 / servo pulse range ─────────────────────────────────────────────
#define SERVOMIN 150
#define SERVOMAX 500
#define SERVOM_FREQ 50

#define CH_L_MG996R 0
#define CH_L_SG90 1
#define CH_R_MG996R 2
#define CH_R_SG90 3

#define IDX_L_SG90 1
#define IDX_R_SG90 3
#define SG90_SLOT_COUNT 2

static const uint8_t CH_MAP[4] = {CH_L_MG996R, CH_L_SG90, CH_R_MG996R, CH_R_SG90};

// ── Timing ───────────────────────────────────────────────────────────────────
#define SERVO_TICK_MS 20UL       // 50 Hz motion loop
#define TIMEOUT_MS 30000UL
#define DEBUG_ACK 0

// ── MG996R (metal gear — keep existing smooth ramp) ─────────────────────────
#define SERVO_EMA_ALPHA 0.12f
#define SNAP_DEG 0.25f
#define PWM_DEADBAND_MG 3

// ── SG90 (plastic gear — anti-jitter) ───────────────────────────────────────
#define SG90_TARGET_DEADBAND_DEG 2.0f   // Ignore serial targets within ±2° of active goal
#define SG90_ASLEEP_WAKE_DEG 3.0f       // Deliberate move required to exit PWM-off sleep
#define SG90_AT_TARGET_DEG 0.4f           // Consider "arrived" for sleep timer
#define SG90_MAX_STEP_DEG 1.0f            // Interpolation cap per tick
#define SG90_STEP_GAIN 0.14f              // Fraction of error applied per tick
#define SG90_HOME_DEG 92.0f
#define SG90_SLEEP_DELAY_MS 400UL         // Hold at target, then PWM off
#define PWM_DEADBAND_SG90 8               // Skip tiny PCA9685 writes while moving

// ── Shared state ─────────────────────────────────────────────────────────────
float targetDeg[4] = {0.0f, SG90_HOME_DEG, 180.0f, SG90_HOME_DEG};
float currentDeg[4] = {0.0f, SG90_HOME_DEG, 180.0f, SG90_HOME_DEG};
int lastPwm[4] = {-1, -1, -1, -1};

bool sg90Asleep[SG90_SLOT_COUNT] = {false, false};
bool sg90AtTarget[SG90_SLOT_COUNT] = {true, true};
unsigned long sg90AtTargetSince[SG90_SLOT_COUNT] = {0, 0};

char lineBuf[64];
uint8_t lineLen = 0;
unsigned long lastCommandMs = 0;
unsigned long lastServoTickMs = 0;
bool isAtHome = true;

// ── Helpers ──────────────────────────────────────────────────────────────────
static bool isSg90Channel(uint8_t idx) {
  return idx == IDX_L_SG90 || idx == IDX_R_SG90;
}

static uint8_t sg90Slot(uint8_t idx) {
  return (idx == IDX_L_SG90) ? 0 : 1;
}

static int pwmDeadband(uint8_t idx) {
  return isSg90Channel(idx) ? PWM_DEADBAND_SG90 : PWM_DEADBAND_MG;
}

static int angleToPwm(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return (int)map((long)lround(deg), 0L, 180L, SERVOMIN, SERVOMAX);
}

// PCA9685: on=0 → output driver off (servo relaxes, stops holding / buzzing)
static void disableSg90Output(uint8_t idx, uint8_t slot) {
  pwm.setPWM(CH_MAP[idx], 0, 0);
  lastPwm[idx] = -1;
  sg90Asleep[slot] = true;
}

static void wakeSg90(uint8_t slot) {
  sg90Asleep[slot] = false;
  sg90AtTarget[slot] = false;
}

static void applyChannel(uint8_t idx) {
  uint8_t slot = 0;
  if (isSg90Channel(idx)) {
    slot = sg90Slot(idx);
    if (sg90Asleep[slot]) {
      return;
    }
  }

  int pwmVal = angleToPwm(currentDeg[idx]);
  if (lastPwm[idx] >= 0 && abs(pwmVal - lastPwm[idx]) < pwmDeadband(idx)) {
    return;
  }
  lastPwm[idx] = pwmVal;
  pwm.setPWM(CH_MAP[idx], 0, pwmVal);
}

static void quantizeSg90Targets(float *a) {
  a[IDX_L_SG90] = roundf(a[IDX_L_SG90]);
  a[IDX_R_SG90] = roundf(a[IDX_R_SG90]);
}

static bool sg90TargetIsSignificant(uint8_t idx, float newDeg) {
  uint8_t slot = sg90Slot(idx);
  float q = roundf(newDeg);

  // Always accept homing packets (bypass deadband / asleep wake thresholds)
  if (fabsf(q - SG90_HOME_DEG) <= 0.5f) {
    return fabsf(q - roundf(currentDeg[idx])) >= 0.5f;
  }

  // Asleep: require a deliberate move before re-enabling PWM
  if (sg90Asleep[slot]) {
    return fabsf(q - roundf(currentDeg[idx])) >= SG90_ASLEEP_WAKE_DEG;
  }

  // Awake: reject updates within deadband of the active commanded target
  return fabsf(q - targetDeg[idx]) >= SG90_TARGET_DEADBAND_DEG;
}

static void setTargets(float *a) {
  quantizeSg90Targets(a);

  for (uint8_t i = 0; i < 4; i++) {
    if (isSg90Channel(i)) {
      if (!sg90TargetIsSignificant(i, a[i])) {
        continue;
      }
      uint8_t slot = sg90Slot(i);
      if (sg90Asleep[slot]) {
        wakeSg90(slot);
      } else {
        sg90AtTarget[slot] = false;
      }
      targetDeg[i] = roundf(a[i]);
      continue;
    }
    targetDeg[i] = a[i];
  }

  lastCommandMs = millis();
  isAtHome = false;
}

// ── MG996R ramp (unchanged logic) ───────────────────────────────────────────
static void tickMg996r(uint8_t idx) {
  float err = targetDeg[idx] - currentDeg[idx];
  if (fabsf(err) <= SNAP_DEG) {
    if (currentDeg[idx] != targetDeg[idx]) {
      currentDeg[idx] = targetDeg[idx];
      applyChannel(idx);
    }
    return;
  }
  currentDeg[idx] += err * SERVO_EMA_ALPHA;
  applyChannel(idx);
}

// ── SG90: stepped interpolation + at-target tracking for auto-sleep ─────────
static void tickSg90(uint8_t idx) {
  uint8_t slot = sg90Slot(idx);
  float t = roundf(targetDeg[idx]);
  float err = t - currentDeg[idx];

  if (sg90Asleep[slot]) {
    return;
  }

  if (fabsf(err) < SG90_AT_TARGET_DEG) {
    if (fabsf(currentDeg[idx] - t) > 0.01f) {
      currentDeg[idx] = t;
      applyChannel(idx);
    }
    if (!sg90AtTarget[slot]) {
      sg90AtTarget[slot] = true;
      sg90AtTargetSince[slot] = millis();
    }
    return;
  }

  // Hysteresis: only leave "at target" if error exceeds deadband (avoids timer resets)
  if (fabsf(err) >= SG90_TARGET_DEADBAND_DEG) {
    sg90AtTarget[slot] = false;
  }

  float step = err * SG90_STEP_GAIN;
  if (fabsf(step) > SG90_MAX_STEP_DEG) {
    step = (err > 0.0f) ? SG90_MAX_STEP_DEG : -SG90_MAX_STEP_DEG;
  }
  currentDeg[idx] += step;
  applyChannel(idx);
}

static void tickSg90AutoSleep(unsigned long now) {
  for (uint8_t slot = 0; slot < SG90_SLOT_COUNT; slot++) {
    uint8_t idx = (slot == 0) ? IDX_L_SG90 : IDX_R_SG90;
    if (sg90Asleep[slot] || !sg90AtTarget[slot]) {
      continue;
    }
    if (now - sg90AtTargetSince[slot] < SG90_SLEEP_DELAY_MS) {
      continue;
    }
    disableSg90Output(idx, slot);
  }
}

static void smoothServosTick(unsigned long now) {
  for (uint8_t i = 0; i < 4; i++) {
    if (isSg90Channel(i)) {
      tickSg90(i);
    } else {
      tickMg996r(i);
    }
  }
  tickSg90AutoSleep(now);
}

// ── Serial ───────────────────────────────────────────────────────────────────
static bool anglesValid(const float *a) {
  for (uint8_t i = 0; i < 4; i++) {
    if (a[i] < 0.0f || a[i] > 180.0f) {
      return false;
    }
  }
  return true;
}

static bool parseCompactCsv(const char *line, float *out) {
  int n = sscanf(line, "%f,%f,%f,%f", &out[0], &out[1], &out[2], &out[3]);
  return n == 4;
}

static bool parseLegacy(const char *line, float *out) {
  const char *p = line;
  const char *keys[4] = {"L_M:", ",L_S:", ",R_M:", ",R_S:"};
  const int offsets[4] = {4, 5, 5, 5};
  const char *starts[4];
  starts[0] = strstr(p, keys[0]);
  starts[1] = strstr(p, keys[1]);
  starts[2] = strstr(p, keys[2]);
  starts[3] = strstr(p, keys[3]);
  if (!starts[0] || !starts[1] || !starts[2] || !starts[3]) {
    return false;
  }
  out[0] = atof(starts[0] + offsets[0]);
  out[1] = atof(starts[1] + offsets[1]);
  out[2] = atof(starts[2] + offsets[2]);
  out[3] = atof(starts[3] + offsets[3]);
  return true;
}

static void handleLine() {
  lineBuf[lineLen] = '\0';
  float parsed[4];

  bool ok = parseCompactCsv(lineBuf, parsed);
  if (!ok) {
    ok = parseLegacy(lineBuf, parsed);
  }
  if (!ok || !anglesValid(parsed)) {
#if DEBUG_ACK
    Serial.println(F("[ESP32 ERROR] Bad packet"));
#endif
    return;
  }

  setTargets(parsed);

#if DEBUG_ACK
  Serial.print(F("[ESP32 ACK] "));
  Serial.print(parsed[0], 1);
  Serial.print(',');
  Serial.print(parsed[1], 1);
  Serial.print(',');
  Serial.print(parsed[2], 1);
  Serial.print(',');
  Serial.println(parsed[3], 1);
#endif
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        handleLine();
        lineLen = 0;
      }
      continue;
    }
    if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

static void returnHome() {
  float home[4] = {0.0f, SG90_HOME_DEG, 180.0f, SG90_HOME_DEG};
  setTargets(home);
  isAtHome = true;
#if DEBUG_ACK
  Serial.println(F("[ESP32] Watchdog — homing targets"));
#endif
}

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVOM_FREQ);
  delay(10);

  for (uint8_t i = 0; i < 4; i++) {
    applyChannel(i);
  }
  lastCommandMs = millis();
  lastServoTickMs = millis();

  Serial.println(F("[ESP32] Ready — MG996R ramp | SG90 deadband+sleep"));
}

void loop() {
  pollSerial();

  unsigned long now = millis();
  if (now - lastServoTickMs >= SERVO_TICK_MS) {
    lastServoTickMs = now;
    smoothServosTick(now);
  }

  if (!isAtHome && (now - lastCommandMs >= TIMEOUT_MS)) {
    returnHome();
    lastCommandMs = now;
  }
}
