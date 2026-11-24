/*
 * Self-Balancing Robot v4 — Arduino Uno + MPU6050 + L298N
 * Built for a TALL robot (high centre of mass).
 *
 * LIVE TUNING: open Serial Monitor at 115200, set line ending to "Newline",
 * and type a command, e.g.  p20   d0.8   m   w
 *
 *   p<num>  Kp                 d<num>  Kd               i<num>  Ki
 *   k<num>  speed loop (stops it running away one way)  e.g. k0.01
 *   a<num>  auto-trim rate (slowly fixes leaning)       e.g. a0.0015
 *   o<num>  manual balance offset in degrees            e.g. o-1.5
 *   b<num>  motor deadband (min PWM that turns wheels)  x<num> max PWM
 *   l<num>  left motor trim    r<num>  right motor trim (e.g. 0.95)
 *   m       flip motor direction        g       flip gyro direction
 *   v       plot on/off    ?  show settings    w  SAVE    z  reset defaults
 *
 * Settings saved with "w" are kept in EEPROM, so they survive power-off.
 */

#include <Wire.h>
#include <EEPROM.h>

// ===================== WIRING (same as before) =====================
const int IN1 = 3;
const int IN2 = 2;
const int IN3 = 4;
const int IN4 = 5;
const int ENA = 9;
const int ENB = 10;
const int MPU_ADDR = 0x68;

// ===================== SETTINGS (tunable live) =====================
struct Settings {
  uint16_t magic;
  float Kp, Ki, Kd;
  float Ks;          // speed loop: degrees of lean-back per PWM of average drive
  float trimRate;    // auto-trim speed of the balance point
  float offset;      // manual balance-point offset (deg)
  float leftTrim, rightTrim;
  int   deadband;
  int   maxPwm;
  int8_t motorDir;   // +1 / -1
  int8_t gyroSign;   // +1 / -1
};
const uint16_t MAGIC = 0xB0B4;
Settings S;

void loadDefaults() {
  S.magic = MAGIC;
  // Tall robot = falls slower, so start gentle and rely more on Kd.
  S.Kp = 19.0;
  S.Ki = 300.0;
  S.Kd = 1.2;
  S.Ks = 0.008;          // turn on after it balances (try k0.008)
  S.trimRate = 0.0015;    // turn on after it balances (try a0.0015)
  S.offset = 0.0;
  S.leftTrim = 1.0;
  S.rightTrim = 1.0;
  S.deadband = 30;
  S.maxPwm = 245;
  S.motorDir = 1;
  S.gyroSign = 1;
}

// ===================== FIXED CONSTANTS =====================
const unsigned long LOOP_US = 5000;   // 200 Hz
const float DT = LOOP_US / 1e6;
// Tall robot: the accelerometer sits high up and gets shaken whenever the
// wheels accelerate, so trust the gyro more than usual.
const float ALPHA = 0.996;
const float FALL_ANGLE = 35.0;   // motors off past this
const float ARM_ANGLE  = 3.0;    // motors only start when held within 3° of upright
const float I_LIMIT    = 60.0;   // max PWM from the I term
const float AVG_TAU    = 0.8;    // averaging time for drift detection (s)
const float TRIM_LIMIT = 5.0;    // auto-trim can move balance point at most ±5°
const float DB_RAMP    = 8.0;    // fade the deadband in smoothly (stops buzzing)

// ===================== STATE =====================
float gyroBias = 0;
float baseAngle = 0;
float angle = 0;
float trim = 0;
float integral = 0;
float avgOut = 0;
float output = 0;
bool  armed = false;
bool  plot = true;
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

// accAngle in degrees, gyroRaw in deg/s (no sign/bias applied)
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
    float boost = S.deadband * min(mag / DB_RAMP, 1.0f);
    pwm = (int)(boost + mag * (S.maxPwm - S.deadband) / (float)S.maxPwm);
    pwm = constrain(pwm, 0, S.maxPwm);
  }
  digitalWrite(inA, cmd > 0 && pwm > 0);
  digitalWrite(inB, cmd < 0 && pwm > 0);
  analogWrite(en, pwm);
}

void driveMotors(float cmd) {
  cmd *= S.motorDir;
  setMotor(ENA, IN2, IN1, cmd * S.leftTrim);
  setMotor(ENB, IN4, IN3, cmd * S.rightTrim);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ===================== SERIAL COMMANDS =====================
void printSettings() {
  Serial.println(F("---- settings ----"));
  Serial.print(F("Kp="));  Serial.print(S.Kp, 2);
  Serial.print(F(" Ki=")); Serial.print(S.Ki, 2);
  Serial.print(F(" Kd=")); Serial.println(S.Kd, 3);
  Serial.print(F("speed k=")); Serial.print(S.Ks, 4);
  Serial.print(F(" autotrim a=")); Serial.print(S.trimRate, 4);
  Serial.print(F(" offset o=")); Serial.println(S.offset, 2);
  Serial.print(F("deadband b=")); Serial.print(S.deadband);
  Serial.print(F(" maxPwm x=")); Serial.print(S.maxPwm);
  Serial.print(F(" trims l=")); Serial.print(S.leftTrim, 2);
  Serial.print(F(" r=")); Serial.println(S.rightTrim, 2);
  Serial.print(F("motorDir=")); Serial.print(S.motorDir);
  Serial.print(F(" gyroSign=")); Serial.println(S.gyroSign);
  Serial.print(F("current trim=")); Serial.println(trim, 2);
}

void runCommand(char *c) {
  char k = c[0];
  float v = atof(c + 1);
  switch (k) {
    case 'p': S.Kp = v; break;
    case 'i': S.Ki = v; integral = 0; break;
    case 'd': S.Kd = v; break;
    case 'k': S.Ks = v; break;
    case 'a': S.trimRate = v; break;
    case 'o': S.offset = v; break;
    case 'b': S.deadband = constrain((int)v, 0, 150); break;
    case 'x': S.maxPwm = constrain((int)v, 50, 255); break;
    case 'l': S.leftTrim = v; break;
    case 'r': S.rightTrim = v; break;
    case 'm': S.motorDir = -S.motorDir; break;
    case 'g': S.gyroSign = -S.gyroSign; break;
    case 'v': plot = !plot; break;
    case 'w': EEPROM.put(0, S); Serial.println(F("SAVED")); break;
    case 'z': loadDefaults(); trim = 0; integral = 0; Serial.println(F("defaults loaded")); break;
    case '?': break;
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

  EEPROM.get(0, S);
  if (S.magic != MAGIC || abs(S.motorDir) != 1 || abs(S.gyroSign) != 1) {
    loadDefaults();
  }

  Wire.begin();
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);   // wake
  mpuWrite(0x1A, 0x03);   // low-pass ~44 Hz
  mpuWrite(0x1B, 0x00);   // gyro ±250 deg/s
  mpuWrite(0x1C, 0x00);   // accel ±2 g
  delay(100);

  Serial.println(F("Calibrating - HOLD THE ROBOT STILL AT ITS BALANCE POINT"));
  const int N = 600;
  float accSum = 0, gyroSum = 0;
  for (int i = 0; i < N; i++) {
    float a, g;
    readIMU(a, g);
    accSum += a;
    gyroSum += g;
    delay(3);
  }
  baseAngle = accSum / N;
  gyroBias  = gyroSum / N;
  angle     = baseAngle;

  Serial.print(F("baseAngle=")); Serial.print(baseAngle, 2);
  Serial.print(F("  gyroBias=")); Serial.println(gyroBias, 3);
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
  float rate = S.gyroSign * (gyroRaw - gyroBias);
  angle = ALPHA * (angle + rate * DT) + (1.0 - ALPHA) * accAngle;

  // ---- target angle ----
  // offset: manual fix, trim: slow automatic fix,
  // Ks term: if it keeps driving one way, lean back against it.
  float target = baseAngle + S.offset + trim - S.Ks * avgOut;
  float error = angle - target;

  if (!armed) {
    stopMotors();
    output = 0;
    if (fabs(angle - (baseAngle + S.offset + trim)) < ARM_ANGLE) {
      armed = true;
      integral = 0;
      avgOut = 0;
    }
  } else if (fabs(error) > FALL_ANGLE) {
    armed = false;               // fell: stop and wait to be stood back up
    stopMotors();
    output = 0;
    integral = 0;
    avgOut = 0;
  } else {
    // ---- PID ----
    integral += S.Ki * error * DT;
    integral = constrain(integral, -I_LIMIT, I_LIMIT);

    output = S.Kp * error + integral + S.Kd * rate;

    if (fabs(output) > S.maxPwm && (output > 0) == (error > 0)) {
      integral -= S.Ki * error * DT;      // anti-windup
    }
    output = constrain(output, -S.maxPwm, S.maxPwm);
    driveMotors(output);

    // ---- drift detection ----
    avgOut += (output - avgOut) * (DT / AVG_TAU);
    if (fabs(error) < 6.0) {
      trim -= S.trimRate * avgOut * DT;
      trim = constrain(trim, -TRIM_LIMIT, TRIM_LIMIT);
    }
  }

  // ---- plot ~10 Hz ----
  if (plot && ++printCount >= 20) {
    printCount = 0;
    Serial.print(F("acc:"));     Serial.print(accAngle, 1);
    Serial.print(F(",angle:"));  Serial.print(angle, 1);
    Serial.print(F(",target:")); Serial.print(target, 1);
    Serial.print(F(",out:"));    Serial.println(output / 10.0, 1);  // scaled to fit the plot
  }
}