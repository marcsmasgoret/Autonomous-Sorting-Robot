// Milestone 4 Full Sorting System Code
// Lab 003, Group 001
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <math.h>

// -------------------- Servo setup --------------------------
const int cServoPin = 4;
const long cMinDutyCycle = 400;
const long cMaxDutyCycle = 2100;

// Servo positions
const int SERVO_POS_DEFAULT = 0;    // non-green direction
const int SERVO_POS_GREEN   = 90;   // green direction

enum ServoState { SERVO_AT_DEFAULT, SERVO_MOVING_TO_GREEN, SERVO_AT_GREEN, SERVO_MOVING_TO_DEFAULT };
ServoState servoState = SERVO_AT_DEFAULT;

// Debounce timers
uint32_t greenSeenAt    = 0;
uint32_t notGreenSeenAt = 0;
bool greenConfirmed     = false;
const uint32_t CONFIRM_MS = 1000;

// -------------------- I2C pins -----------------------------
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

// -------------------- Color sensor -------------------------
Adafruit_TCS34725 tcs =
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

static const uint8_t AVG_SAMPLES = 3;
static const uint16_t LOOP_DELAY_MS = 25;

float GREEN_MIN_NORM = 0.40f;
float RED_MAX_NORM   = 0.30f;
float BLUE_MAX_NORM  = 0.32f;
float HUE_MIN_DEG    = 115.0f;
float HUE_MAX_DEG    = 135.0f;
float DOMINANCE_MIN  = 0.10f;
uint16_t CLEAR_MIN   = 150;

// -------------------- Motor setup --------------------------
const int M1_IN1 = 26;
const int M1_IN2 = 27;
const int M1_PWM = 25;
const int M2_IN1 = 14;
const int M2_IN2 = 16;
const int M2_PWM = 17;

const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

const int M1_SPEED = 240;
const int M2_SPEED = 255;

const unsigned long RUN_TIME  = 500;   // how long each motor runs
const unsigned long GAP_TIME  = 0;   // pause between motors

enum MotorState { M1_RUNNING, GAP_TO_M2, M2_RUNNING, GAP_TO_M1 };
MotorState motorState = M1_RUNNING;
unsigned long motorStateStart = 0;

// -------------------- Data structure -----------------------
struct ColorReading {
  uint16_t r, g, b, c;
  float rn, gn, bn;
  float hue, sat, val;
  float dominance;
};

// -------------------- Utility functions --------------------
void rgbToHsv(float r, float g, float b, float &h, float &s, float &v) {
  float maxVal = fmaxf(r, fmaxf(g, b));
  float minVal = fminf(r, fminf(g, b));
  float delta = maxVal - minVal;
  v = maxVal;
  if (maxVal <= 0.0f) { s = 0.0f; h = 0.0f; return; }
  s = delta / maxVal;
  if (delta <= 0.000001f) { h = 0.0f; return; }
  if (maxVal == r)      h = 60.0f * fmodf(((g - b) / delta), 6.0f);
  else if (maxVal == g) h = 60.0f * (((b - r) / delta) + 2.0f);
  else                  h = 60.0f * (((r - g) / delta) + 4.0f);
  if (h < 0.0f) h += 360.0f;
}

bool hueInGreenRange(float h) {
  return (h >= HUE_MIN_DEG && h <= HUE_MAX_DEG);
}

ColorReading readAveragedColor() {
  uint32_t rSum = 0, gSum = 0, bSum = 0, cSum = 0;
  for (uint8_t i = 0; i < AVG_SAMPLES; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    rSum += r; gSum += g; bSum += b; cSum += c;
    delay(5);
  }
  ColorReading out;
  out.r = rSum / AVG_SAMPLES;
  out.g = gSum / AVG_SAMPLES;
  out.b = bSum / AVG_SAMPLES;
  out.c = cSum / AVG_SAMPLES;
  float sumRGB = (float)out.r + (float)out.g + (float)out.b;
  if (sumRGB < 1.0f) sumRGB = 1.0f;
  out.rn = out.r / sumRGB;
  out.gn = out.g / sumRGB;
  out.bn = out.b / sumRGB;
  rgbToHsv(out.rn, out.gn, out.bn, out.hue, out.sat, out.val);
  out.dominance = out.gn - fmaxf(out.rn, out.bn);
  return out;
}

bool isGreenEraser(const ColorReading &x) {
  if (x.c < CLEAR_MIN)             return false;
  if (x.gn < GREEN_MIN_NORM)       return false;
  if (x.rn > RED_MAX_NORM)         return false;
  if (x.bn > BLUE_MAX_NORM)        return false;
  if (x.dominance < DOMINANCE_MIN) return false;
  if (!hueInGreenRange(x.hue))     return false;
  return true;
}

void printHeader() {
  Serial.println();
  Serial.println("r,g,b,c,rNorm,gNorm,bNorm,hue,sat,val,dominance,isGreen,servoState");
  Serial.println("--------------------------------------------------------------------");
}

void printReading(const ColorReading &x, bool greenNow) {
  Serial.print(x.r);            Serial.print(",");
  Serial.print(x.g);            Serial.print(",");
  Serial.print(x.b);            Serial.print(",");
  Serial.print(x.c);            Serial.print(",");
  Serial.print(x.rn, 4);        Serial.print(",");
  Serial.print(x.gn, 4);        Serial.print(",");
  Serial.print(x.bn, 4);        Serial.print(",");
  Serial.print(x.hue, 1);       Serial.print(",");
  Serial.print(x.sat, 3);       Serial.print(",");
  Serial.print(x.val, 3);       Serial.print(",");
  Serial.print(x.dominance, 4); Serial.print(",");
  Serial.print(greenNow ? 1 : 0); Serial.print(",");
  Serial.println(servoState);
}

long degreesToDutyCycle(int deg) {
  return map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);
}

void moveServoTo(int deg) {
  ledcWrite(cServoPin, degreesToDutyCycle(deg));
}

// -------------------- Motor functions ----------------------
void turnMotor1On() {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  ledcWrite(M1_PWM, M1_SPEED);
}
void turnMotor1Off() {
  ledcWrite(M1_PWM, 0);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
}
void turnMotor2On() {
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
  ledcWrite(M2_PWM, M2_SPEED);
}
void turnMotor2Off() {
  ledcWrite(M2_PWM, 0);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}

// -------------------- Motor state machine ------------------
void updateMotors() {
  unsigned long now = millis();
  switch (motorState) {
    case M1_RUNNING:
      if (now - motorStateStart >= RUN_TIME) {
        turnMotor1Off();
        motorStateStart = now;
        motorState = GAP_TO_M2;
      }
      break;
    case GAP_TO_M2:
      if (now - motorStateStart >= GAP_TIME) {
        turnMotor2On();
        motorStateStart = now;
        motorState = M2_RUNNING;
      }
      break;
    case M2_RUNNING:
      if (now - motorStateStart >= RUN_TIME) {
        turnMotor2Off();
        motorStateStart = now;
        motorState = GAP_TO_M1;
      }
      break;
    case GAP_TO_M1:
      if (now - motorStateStart >= GAP_TIME) {
        turnMotor1On();
        motorStateStart = now;
        motorState = M1_RUNNING;
      }
      break;
  }
}

// -------------------- Setup --------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Color sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!tcs.begin()) {
    Serial.println("ERROR: Color sensor not found. Check wiring.");
    while (true) { delay(100); }
  }
  Serial.println("Color sensor found.");
  printHeader();

  // Servo — start at non-green position
  pinMode(cServoPin, OUTPUT);
  ledcAttach(cServoPin, 50, 14);
  moveServoTo(SERVO_POS_DEFAULT);

  // Motors
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, LOW);

  bool ok1 = ledcAttach(M1_PWM, PWM_FREQ, PWM_RES);
  bool ok2 = ledcAttach(M2_PWM, PWM_FREQ, PWM_RES);
  if (!ok1 || !ok2) {
    Serial.println("PWM attach failed");
    while (true) {}
  }

  // Start with motor 1
  turnMotor1On();
  motorStateStart = millis();
}

// -------------------- Loop ---------------------------------
void loop() {
  uint32_t now = millis();

  // --- Color reading ---
  ColorReading x = readAveragedColor();
  bool greenNow = isGreenEraser(x);
  printReading(x, greenNow);

  // --- Servo logic ---
  if (servoState == SERVO_AT_DEFAULT) {
    if (greenNow) {
      if (greenSeenAt == 0) greenSeenAt = now;
      if (now - greenSeenAt >= CONFIRM_MS) {
        moveServoTo(SERVO_POS_GREEN);
        servoState = SERVO_AT_GREEN;
        greenConfirmed = true;
        notGreenSeenAt = 0;
        Serial.println(">>> SERVO → GREEN");
      }
    } else {
      greenSeenAt = 0;
    }
  }
  else if (servoState == SERVO_AT_GREEN) {
    if (!greenNow) {
      if (notGreenSeenAt == 0) notGreenSeenAt = now;
      if (now - notGreenSeenAt >= CONFIRM_MS) {
        moveServoTo(SERVO_POS_DEFAULT);
        servoState = SERVO_AT_DEFAULT;
        greenSeenAt = 0;
        Serial.println(">>> SERVO → DEFAULT");
      }
    } else {
      notGreenSeenAt = 0;
    }
  }

  // --- Motor state machine ---
  updateMotors();

  delay(LOOP_DELAY_MS);
}

