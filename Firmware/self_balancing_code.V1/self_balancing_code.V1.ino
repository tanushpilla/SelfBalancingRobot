/*
 * Self-Balancing Robot — Arduino Uno + MPU6050 + L298N
 *
 * Fixes "keeps drifting / falling toward one side":
 *  1. Gyro bias calibration at startup (a biased gyro makes the angle creep).
 *  2. Balance-point calibration at startup (hold the robot at its balance point).
 *  3. ADAPTIVE BALANCE POINT: the code watches the average motor command.
 *     If the motors keep pushing the same direction, the robot is leaning
 *     toward that side, so the target angle is slowly shifted the opposite way
 *     until the average push goes back to ~0.
 *  4. Integral anti-windup and reset, so the integral doesn't build up and
 *     overshoot to one side.
 *  5. Motor deadband compensation and left/right trim, so a weaker motor
 *     doesn't pull the robot to one side.
 *
 * No libraries needed except Wire.
 */

#include <Wire.h>

// ---------------- PINS (L298N) — change to match your wiring ----------------
int IN1 = 3;
int IN2 = 2;
int IN3 = 4;
int IN4 = 5;

int ENA = 9;
int ENB = 10;

// ---------------- MPU6050 ----------------
const int MPU_ADDR = 0x68;

// Axis setup: depends on how the IMU is mounted.
// Default: board mounted upright, X axis points forward, Z axis points up.
// Turn on DEBUG, tilt the robot forward by hand: "angle" should go POSITIVE
// and "rate" should be positive while you tilt forward. If not, flip the signs.
const float ANGLE_SIGN = 1.0;   // set to -1.0 if angle goes the wrong way
const float GYRO_SIGN  = 1.0;   // set to -1.0 if rate has the opposite sign to angle

// ---------------- PID gains (start here, then tune) ----------------
// Lower starting values: Kp 30 + full 11.1 V was making it shake violently.
// Tune Kp and Kd first with Ki = 0 and TRIM_RATE = 0, then add those back.
float Kp = 18.0;    // PWM per degree
float Ki = 0.0;     // PWM per (degree * second)  -> try 1.0 once Kp/Kd are good
float Kd = 1.5;     // PWM per (degree / second)

// ---------------- Drift / overcompensation correction ----------------
const float TRIM_RATE      = 0.0;    // turn on (0.0015) AFTER Kp/Kd are tuned; 0.005 was too fast
const float TRIM_LIMIT     = 4.0;    // max shift from the calibrated balance point (deg)
const float AVG_TAU        = 1.0;    // averaging time for motor output (s)
const float TRIM_WINDOW    = 6.0;    // only adapt when within this many degrees of upright
const float INTEGRAL_LIMIT = 80.0;   // max PWM contribution from the I term

// ---------------- Motors ----------------
const int   MOTOR_DEADBAND = 35;   // min PWM that actually makes the wheels turn (measure yours)
const int   MAX_PWM    = 180;      // cap: full 255 on an 11.1 V LiPo is very violent
const float DB_RAMP    = 8.0;      // fade the deadband in over the first few PWM (stops buzzing)
const float LEFT_TRIM  = 1.00;     // if the robot turns while balancing, lower the stronger side
const float RIGHT_TRIM = 1.00;     // e.g. LEFT_TRIM = 0.93
const float FALL_ANGLE = 40.0;     // stop the motors past this tilt

// ---------------- Timing ----------------
const unsigned long LOOP_US = 5000; // 200 Hz
const float DT = LOOP_US / 1e6;
const float ALPHA = 0.98;           // complementary filter: trust in gyro

#define DEBUG 1   // prints at 10 Hz for Serial Plotter; set to 0 once tuned

// ---------------- State ----------------
float gyroBias = 0;
float angle = 0;
float baseSetpoint = 0;   // calibrated balance point
float setpoint = 0;       // adaptive balance point
float integral = 0;
float avgOutput = 0;      // low-passed motor command = "which way is it drifting"
unsigned long lastLoop = 0;

// ---------------------------------------------------------------------------
void mpuWrite(byte reg, byte val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Reads accel angle (deg) and gyro rate (deg/s, raw, before bias removal)
void readIMU(float &accAngle, float &gyroRate) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                       // temperature (unused)
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  (void)ay; (void)gx; (void)gz;

  accAngle = ANGLE_SIGN * atan2((float)ax, (float)az) * 180.0 / PI;
  gyroRate = GYRO_SIGN * ANGLE_SIGN * gy / 131.0;  // ±250 deg/s range
}

void setMotor(int en, int inA, int inB, float cmd) {
  // Old version jumped straight from 0 to ±35 for tiny commands, so near
  // upright the motors slammed back and forth (= violent shaking).
  // Now the deadband boost fades in over the first DB_RAMP units.
  float mag = fabs(cmd);
  int pwm = 0;
  if (mag > 0.5) {
    float boost = MOTOR_DEADBAND * min(mag / DB_RAMP, 1.0f);
    pwm = (int)(boost + mag * (MAX_PWM - MOTOR_DEADBAND) / (float)MAX_PWM);
    pwm = constrain(pwm, 0, MAX_PWM);
    if (cmd < 0) pwm = -pwm;
  }

  digitalWrite(inA, pwm > 0);
  digitalWrite(inB, pwm < 0);
  analogWrite(en, abs(pwm));
}

void driveMotors(float cmd) {
  setMotor(ENA, IN2, IN1, cmd * LEFT_TRIM);
  setMotor(ENB, IN4, IN3, cmd * RIGHT_TRIM);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  Wire.begin();
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);   // wake up
  mpuWrite(0x1A, 0x03);   // digital low-pass filter ~44 Hz
  mpuWrite(0x1B, 0x00);   // gyro ±250 deg/s
  mpuWrite(0x1C, 0x00);   // accel ±2 g
  delay(100);

  // HOLD THE ROBOT STILL AT ITS BALANCE POINT during these ~2 seconds.
  Serial.println(F("Calibrating: hold robot upright at its balance point..."));
  const int N = 500;
  float gyroSum = 0, accSum = 0;
  for (int i = 0; i < N; i++) {
    float a, g;
    readIMU(a, g);
    gyroSum += g;
    accSum  += a;
    delay(4);
  }
  gyroBias     = gyroSum / N;
  baseSetpoint = accSum / N;
  setpoint     = baseSetpoint;
  angle        = baseSetpoint;

  Serial.print(F("Gyro bias: "));     Serial.println(gyroBias, 3);
  Serial.print(F("Balance point: ")); Serial.println(baseSetpoint, 2);
  lastLoop = micros();
}

// ---------------------------------------------------------------------------
void loop() {
  if (micros() - lastLoop < LOOP_US) return;
  lastLoop += LOOP_US;

  // --- Angle estimate ---
  float accAngle, gyroRaw;
  readIMU(accAngle, gyroRaw);
  float rate = gyroRaw - gyroBias;
  angle = ALPHA * (angle + rate * DT) + (1.0 - ALPHA) * accAngle;

  float error = angle - setpoint;   // positive = leaning forward

  // --- Fallen over: stop and reset everything that accumulates ---
  if (fabs(error) > FALL_ANGLE) {
    stopMotors();
    integral = 0;
    avgOutput = 0;
    return;
  }

  // --- PID ---
  integral += Ki * error * DT;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);  // anti-windup

  // Derivative uses the gyro directly (cleaner than differentiating the angle)
  float output = Kp * error + integral + Kd * rate;
  output = constrain(output, -MAX_PWM, MAX_PWM);

  // If the output is saturated, stop the integral from growing further
  // in that direction (prevents overshoot to the other side).
  if ((output >= MAX_PWM && error > 0) || (output <= -MAX_PWM && error < 0)) {
    integral -= Ki * error * DT;
  }

  driveMotors(output);

  // --- Drift detection & correction ---
  // avgOutput tells us which way the motors have been pushing on average.
  // Consistently positive = constantly driving forward = the robot is leaning
  // forward of its true balance point. So move the setpoint back (and vice versa).
  avgOutput += (output - avgOutput) * (DT / AVG_TAU);

  if (fabs(error) < TRIM_WINDOW) {
    setpoint -= TRIM_RATE * avgOutput * DT;
    setpoint = constrain(setpoint, baseSetpoint - TRIM_LIMIT, baseSetpoint + TRIM_LIMIT);
  }

#if DEBUG
  static int printCount = 0;
  if (++printCount >= 20) {   // ~10 Hz
    printCount = 0;
    Serial.print(F("angle:"));   Serial.print(angle, 2);
    Serial.print(F(",set:"));    Serial.print(setpoint, 2);
    Serial.print(F(",rate:"));   Serial.print(rate, 1);
    Serial.print(F(",out:"));    Serial.print(output, 0);
    Serial.print(F(",avg:"));    Serial.println(avgOutput, 0);
  }
#endif
}