#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Standard 12-bit PWM values for servos @ 50Hz
#define SERVOMIN 150 // 0 degrees (approx. 0.5ms pulse)
#define SERVOMAX 500 // 180 degrees (approx. 2.5ms pulse)
#define SERVOM_FREQ 50

// PCA9685 Channels
#define CH_L_MG996R 0 // Left Shoulder Pitch
#define CH_L_SG90   1 // Left Elbow Yaw
#define CH_R_MG996R 2 // Right Shoulder Pitch
#define CH_R_SG90   3 // Right Elbow Yaw

// Neutral starting positions
const float L_MG996R_NEUTRAL = 0.0;
const float L_SG90_NEUTRAL   = 90.0;
const float R_MG996R_NEUTRAL = 0.0;
const float R_SG90_NEUTRAL   = 90.0;

// Watchdog variables
unsigned long lastCommandTime = 0;
bool isAtHome = true;
const unsigned long TIMEOUT_MS = 15000; // 15 seconds

// Setup function
void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT); // Onboard LED status

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVOM_FREQ);
  delay(10);

  // Write starting positions
  int pwm_l_mg = map(L_MG996R_NEUTRAL, 0, 180, SERVOMIN, SERVOMAX);
  int pwm_l_sg = map(L_SG90_NEUTRAL, 0, 180, SERVOMIN, SERVOMAX);
  int pwm_r_mg = map(R_MG996R_NEUTRAL, 0, 180, SERVOMIN, SERVOMAX);
  int pwm_r_sg = map(R_SG90_NEUTRAL, 0, 180, SERVOMIN, SERVOMAX);

  pwm.setPWM(CH_L_MG996R, 0, pwm_l_mg);
  pwm.setPWM(CH_L_SG90, 0, pwm_l_sg);
  pwm.setPWM(CH_R_MG996R, 0, pwm_r_mg);
  pwm.setPWM(CH_R_SG90, 0, pwm_r_sg);

  lastCommandTime = millis();
  isAtHome = true;

  Serial.println(
      "[ESP32] Dual-Arm Driver Initialized. Awaiting serial commands (115200 baud)...");
}

// Loop function to read serial commands and update servos
void loop() {
  // Read serial command
  if (Serial.available() > 0) {
    // Read full command packet ending with newline
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Check if indices match L_M:<left_pitch>,L_S:<left_yaw>,R_M:<right_pitch>,R_S:<right_yaw>
    int l_m_idx = input.indexOf("L_M:");
    int l_s_idx = input.indexOf(",L_S:");
    int r_m_idx = input.indexOf(",R_M:");
    int r_s_idx = input.indexOf(",R_S:");

    if (l_m_idx != -1 && l_s_idx != -1 && r_m_idx != -1 && r_s_idx != -1) {
      // Extract substrings for angles
      String l_mg_str = input.substring(l_m_idx + 4, l_s_idx);
      String l_sg_str = input.substring(l_s_idx + 5, r_m_idx);
      String r_mg_str = input.substring(r_m_idx + 5, r_s_idx);
      String r_sg_str = input.substring(r_s_idx + 5);

      float l_mg_angle = l_mg_str.toFloat();
      float l_sg_angle = l_sg_str.toFloat();
      float r_mg_angle = r_mg_str.toFloat();
      float r_sg_angle = r_sg_str.toFloat();

      // Validate constraints (0.0 to 180.0 degrees)
      if (l_mg_angle >= 0.0 && l_mg_angle <= 180.0 && 
          l_sg_angle >= 0.0 && l_sg_angle <= 180.0 &&
          r_mg_angle >= 0.0 && r_mg_angle <= 180.0 &&
          r_sg_angle >= 0.0 && r_sg_angle <= 180.0) {

        // Map angles to 12-bit PWM resolution
        int pwm_l_mg = map(l_mg_angle, 0, 180, SERVOMIN, SERVOMAX);
        int pwm_l_sg = map(l_sg_angle, 0, 180, SERVOMIN, SERVOMAX);
        int pwm_r_mg = map(r_mg_angle, 0, 180, SERVOMIN, SERVOMAX);
        int pwm_r_sg = map(r_sg_angle, 0, 180, SERVOMIN, SERVOMAX);

        // Command the PCA9685 driver channels
        pwm.setPWM(CH_L_MG996R, 0, pwm_l_mg);
        pwm.setPWM(CH_L_SG90, 0, pwm_l_sg);
        pwm.setPWM(CH_R_MG996R, 0, pwm_r_mg);
        pwm.setPWM(CH_R_SG90, 0, pwm_r_sg);

        // Update timing and home state
        lastCommandTime = millis();
        isAtHome = false;

        // Blink onboard LED to show active command received
        digitalWrite(2, HIGH);
        delay(10);
        digitalWrite(2, LOW);

        // Send debug acknowledgement back to host PC
        Serial.print("[ESP32 ACK] L:(");
        Serial.print(l_mg_angle, 1);
        Serial.print(", ");
        Serial.print(l_sg_angle, 1);
        Serial.print(") | R:(");
        Serial.print(r_mg_angle, 1);
        Serial.print(", ");
        Serial.print(r_sg_angle, 1);
        Serial.println(")");
      } else {
        Serial.println("[ESP32 ERROR] Angle out of bounds (0-180)!");
      }
    }
  }

  // Inactivity watchdog: if 15 seconds pass without commands, home both physical arms
  if (!isAtHome && (millis() - lastCommandTime >= TIMEOUT_MS)) {
    int pwm_l_mg = map(L_MG996R_NEUTRAL, 0, 180, SERVOMIN, SERVOMAX);
    int pwm_l_sg = map(L_SG90_NEUTRAL, 0, 180, SERVOMIN, SERVOMAX);
    int pwm_r_mg = map(R_MG996R_NEUTRAL, 0, 180, SERVOMIN, SERVOMAX);
    int pwm_r_sg = map(R_SG90_NEUTRAL, 0, 180, SERVOMIN, SERVOMAX);

    pwm.setPWM(CH_L_MG996R, 0, pwm_l_mg);
    pwm.setPWM(CH_L_SG90, 0, pwm_l_sg);
    pwm.setPWM(CH_R_MG996R, 0, pwm_r_mg);
    pwm.setPWM(CH_R_SG90, 0, pwm_r_sg);

    Serial.println(
        "[ESP32] Inactivity timeout! Returning both arms to starting positions.");
    isAtHome = true;
  }
}