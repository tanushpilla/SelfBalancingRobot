/*
 * Self-Balancing Robot — Arduino Uno + MPU6050 + L298N
 * Plain PID controller with a manually set balance point.
 *
 * Serial Monitor at 115200, line ending "Newline".
 * Angle is printed 10x per second (also works in Serial Plotter).
 *
 * Optional live commands (lost on reset — copy good values into the code):
 *   p<num>  Kp      i<num>  Ki      d<num>  Kd
 *   t<num>  balance point (degrees), e.g. t1.5
 */

#include <Wire.h>

// ===================== BALANCE POINT =====================
float balancePoint = 0.0;   // degrees — the angle the robot tries to hold

// ===================== PID GAINS =====================
float Kp = 19.0;
float Ki = 250.0;
float Kd = 1.0;

// ===================== MOTOR SETTINGS =====================
const int   DEADBAND   = 30;    // min PWM that turns the wheels
const int   MAX_PWM    = 245;
const float LEFT_TRIM  = 1.0;
const float RIGHT_TRIM = 1.0;
const int   MOTOR_DIR  = 1;     // change to -1 if wheels drive the wrong way
const int   GYRO_SIGN  = 1;     // change to -1 if gyro direction is reversed

// ===================== WIRING =====================
const int IN1 = 3;
const int IN2 = 2;
const int IN3 = 4;
const int IN4 = 5;
const int ENA = 9;
const int ENB = 10;
const int MPU_ADDR = 0x68;

// ===================== FIXED CONSTANTS =====================
const unsigned long LOOP_US = 5000;   // 200 Hz
const float DT = LOOP_US / 1e6;
const float ALPHA = 0.996;       // complementary filter (trust gyro more)
const float FALL_ANGLE = 35.0;   // motors off past this
const float ARM_ANGLE  = 3.0;    // motors only start within 3° of balance point
const float I_LIMIT    = 60.0;   // max PWM from the I term
const float DB_RAMP    = 8.0;    // fade the deadband in smoothly (stops buzzing)

// ===================== STATE =====================
float angle = 0;
float integral = 0;
float output = 0;
bool  armed = false;
unsigned long lastLoop = 0;
int   printCount = 0;
char  cmdBuf[16];
byte  cmdLen = 0;

// ===================== MPU6050 =====================
void mpuWrite(byte reg, byte val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// accAngle in degrees, gyroRaw in deg/s
void readIMU(float &accAngle, float &gyroRaw) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                        // ay (unused)
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                        // temperature
  Wire.read(); Wire.read();                        // gx
  int16_t gy = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                        // gz

  accAngle = atan2((float)ax, (float)az) * 180.0 / PI;
  gyroRaw  = gy / 131.0;
}

// ===================== MOTORS =====================
void setMotor(int en, int inA, int inB, float cmd) {
  float mag = fabs(cmd);
  int pwm = 0;
  if (mag > 0.5) {
    float boost = DEADBAND * min(mag / DB_RAMP, 1.0f);
    pwm = (int)(boost + mag * (MAX_PWM - DEADBAND) / (float)MAX_PWM);
    pwm = constrain(pwm, 0, MAX_PWM);
  }
  digitalWrite(inA, cmd > 0 && pwm > 0);
  digitalWrite(inB, cmd < 0 && pwm > 0);
  analogWrite(en, pwm);
}

void driveMotors(float cmd) {
  cmd *= MOTOR_DIR;
  setMotor(ENA, IN2, IN1, cmd * LEFT_TRIM);
  setMotor(ENB, IN4, IN3, cmd * RIGHT_TRIM);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ===================== SERIAL COMMANDS =====================
void printSettings() {
  Serial.print(F("Kp="));  Serial.print(Kp, 2);
  Serial.print(F(" Ki=")); Serial.print(Ki, 2);
  Serial.print(F(" Kd=")); Serial.print(Kd, 3);
  Serial.print(F(" balancePoint=")); Serial.println(balancePoint, 2);
}

void runCommand(char *c) {
  char k = c[0];
  float v = atof(c + 1);
  switch (k) {
    case 'p': Kp = v; break;
    case 'i': Ki = v; integral = 0; break;
    case 'd': Kd = v; break;
    case 't': balancePoint = v; integral = 0; break;
    default: Serial.println(F("unknown command")); return;
  }
  printSettings();
}

void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = 0;
        runCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  stopMotors();

  Wire.begin();
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);   // wake
  mpuWrite(0x1A, 0x03);   // low-pass ~44 Hz
  mpuWrite(0x1B, 0x00);   // gyro ±250 deg/s
  mpuWrite(0x1C, 0x00);   // accel ±2 g
  delay(100);

  // Start the angle estimate from the accelerometer
  float a, g;
  readIMU(a, g);
  angle = a;

  printSettings();
  lastLoop = micros();
}

// ===================== LOOP =====================
void loop() {
  handleSerial();

  unsigned long now = micros();
  if (now - lastLoop < LOOP_US) return;
  lastLoop += LOOP_US;
  if (now - lastLoop > LOOP_US) lastLoop = now;   // don't "catch up" after a delay

  // ---- angle ----
  float accAngle, gyroRaw;
  readIMU(accAngle, gyroRaw);
  float rate = GYRO_SIGN * gyroRaw;
  angle = ALPHA * (angle + rate * DT) + (1.0 - ALPHA) * accAngle;

  float error = angle - balancePoint;

  if (!armed) {
    stopMotors();
    output = 0;
    if (fabs(error) < ARM_ANGLE) {
      armed = true;
      integral = 0;
    }
  } else if (fabs(error) > FALL_ANGLE) {
    armed = false;               // fell: stop and wait to be stood back up
    stopMotors();
    output = 0;
    integral = 0;
  } else {
    // ---- PID ----
    integral += Ki * error * DT;
    integral = constrain(integral, -I_LIMIT, I_LIMIT);

    output = Kp * error + integral + Kd * rate;

    if (fabs(output) > MAX_PWM && (output > 0) == (error > 0)) {
      integral -= Ki * error * DT;      // anti-windup
    }
    output = constrain(output, -MAX_PWM, MAX_PWM);
    driveMotors(output);
  }

  // ---- serial output ~10 Hz ----
  if (++printCount >= 20) {
    printCount = 0;
    Serial.print(F("angle:"));   Serial.print(angle, 2);
    Serial.print(F(",acc:"));    Serial.print(accAngle, 2);
    Serial.print(F(",target:")); Serial.print(balancePoint, 2);
    Serial.print(F(",out:"));    Serial.println(output / 10.0, 1);
  }
}