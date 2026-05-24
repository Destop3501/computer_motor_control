/*
 * compuer_sevo_control.ino — Non-blocking dual-arm servo driver (PCA9685)
 *
 * PC protocol (preferred, compact):  "L_MG,L_SG,R_MG,R_SG\n"  e.g. "96,70,83,55\n"
 * Legacy (still supported):        "L_M:96.0,L_S:70.0,R_M:83.0,R_S:55.0\n"
 *
 * The ESP32 ramps current angles toward targets every SERVO_TICK_MS (50 Hz).
 * Serial only updates targets — never blocks the motion loop.
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 500
#define SERVOM_FREQ 50

#define CH_L_MG996R 0
#define CH_L_SG90 1
#define CH_R_MG996R 2
#define CH_R_SG90 3

#define SERVO_TICK_MS 20UL       // 50 Hz onboard interpolation
#define SERVO_EMA_ALPHA 0.18f    // Smooth ramp toward target each tick
#define SNAP_DEG 0.25f
#define TIMEOUT_MS 30000UL         // Must exceed PC hold + keepalive gap
#define DEBUG_ACK 0                // 1 = print ACK (floods serial; slows PC)

static const uint8_t CH_MAP[4] = {CH_L_MG996R, CH_L_SG90, CH_R_MG996R, CH_R_SG90};

float targetDeg[4] = {0.0f, 90.0f, 180.0f, 90.0f};
float currentDeg[4] = {0.0f, 90.0f, 180.0f, 90.0f};

char lineBuf[64];
uint8_t lineLen = 0;

unsigned long lastCommandMs = 0;
unsigned long lastServoTickMs = 0;
bool isAtHome = true;

static int angleToPwm(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return (int)map((long)lround(deg), 0L, 180L, SERVOMIN, SERVOMAX);
}

static void applyChannel(uint8_t idx) {
  pwm.setPWM(CH_MAP[idx], 0, angleToPwm(currentDeg[idx]));
}

static void applyAllChannels() {
  for (uint8_t i = 0; i < 4; i++) {
    applyChannel(i);
  }
}

static bool anglesValid(const float *a) {
  for (uint8_t i = 0; i < 4; i++) {
    if (a[i] < 0.0f || a[i] > 180.0f) {
      return false;
    }
  }
  return true;
}

static void setTargets(const float *a) {
  for (uint8_t i = 0; i < 4; i++) {
    targetDeg[i] = a[i];
  }
  lastCommandMs = millis();
  isAtHome = false;
}

static bool parseCompactCsv(const char *line, float *out) {
  // "96,70,83,55" or "96.0,70.0,83.0,55.0"
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

static void smoothServosTick() {
  for (uint8_t i = 0; i < 4; i++) {
    float err = targetDeg[i] - currentDeg[i];
    if (fabsf(err) <= SNAP_DEG) {
      if (currentDeg[i] != targetDeg[i]) {
        currentDeg[i] = targetDeg[i];
        applyChannel(i);
      }
    } else {
      currentDeg[i] += err * SERVO_EMA_ALPHA;
      applyChannel(i);
    }
  }
}

static void returnHome() {
  targetDeg[0] = 0.0f;
  targetDeg[1] = 90.0f;
  targetDeg[2] = 0.0f;
  targetDeg[3] = 90.0f;
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

  applyAllChannels();
  lastCommandMs = millis();
  lastServoTickMs = millis();

  Serial.println(F("[ESP32] Ready @115200 — compact: L_MG,L_SG,R_MG,R_SG"));
}

void loop() {
  pollSerial();

  unsigned long now = millis();
  if (now - lastServoTickMs >= SERVO_TICK_MS) {
    lastServoTickMs = now;
    smoothServosTick();
  }

  if (!isAtHome && (now - lastCommandMs >= TIMEOUT_MS)) {
    returnHome();
    lastCommandMs = now;
  }
}
