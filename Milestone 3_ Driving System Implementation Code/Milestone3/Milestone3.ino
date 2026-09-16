//
//  MSE 2202 MSEBot base code for Milestone #3
//
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author:   Lab 003 - 1
//  Date:     2025 03 014
//
//  Run mode 0: Robot stopped
//  Run mode 1: Demo sequence (forward, reverse, turn CCW, turn CW, repeat)
//  Run mode 2: Full search / return sequence
//

// Uncomment keywords to enable debugging output
// #define DEBUG_DRIVE_SPEED    1
// #define DEBUG_ULTRASONIC     1
// #define DEBUG_STATE          1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void initUltrasonic(struct Ultrasonic* us);
void ping(struct Ultrasonic* us);
uint32_t getEchoTime(struct Ultrasonic* us);
uint32_t usToCm(uint32_t us);
uint32_t usToIn(uint32_t us);
uint32_t angleToDuty(int angle);
void setServosHome();
void setServosUp();
void initIMU();
void calibrateIMU(uint16_t N = 1500, uint16_t dtMs = 3);
void updateIMU();
void resetYawDeg();
bool turnToRelativeAngle(float targetDeg, int turnDir);
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);
void ARDUINO_ISR_ATTR usTimerISR(void* arg);
void ARDUINO_ISR_ATTR echoISR(void* arg);
void ARDUINO_ISR_ATTR stepperTimerISR();

// Button structure
struct Button {
  const int pin;                    // GPIO pin for button
  volatile uint32_t numberPresses;  // counter for number of button presses
  uint32_t nextPressTime;           // time of next allowable press in milliseconds
  volatile bool pressed;            // flag for button press event
};

// Ultrasonic sensor structure
struct Ultrasonic {
  const int triggerPin;              // GPIO pin for trigger
  const int echoPin;                 // GPIO pin for echo
  volatile uint32_t pulseBegin = 0;  // time of echo pulse start in microseconds
  volatile uint32_t pulseEnd = 0;    // time of echo pulse end in microseconds
  volatile bool newEcho = false;     // new (valid) echo flag
  volatile bool timeout = false;     // echo timeout flag
  hw_timer_t* pTimer = NULL;         // pointer to timer used by trigger interrupt
};

// IMU filter structure
struct AxisFilter {
  float x1 = 0, x2 = 0;
  float y = 0;
};

// Constants
const int cHeartbeatInterval = 75;        // heartbeat update interval, in milliseconds
const int cSmartLED = 23;                 // when DIP switch S1-4 is on, SMART LED is connected to GPIO23
const int cSmartLEDCount = 1;             // number of Smart LEDs in use
const long cDebounceDelay = 170;          // switch debounce delay in milliseconds
const int cNumMotors = 2;                 // Number of DC motors
const int cIN1Pin[] = { 26, 16 };         // GPIO pin(s) for IN1 for left and right motors (A, B)
const int cIN2Pin[] = { 27, 17 };         // GPIO pin(s) for IN2 for left and right motors (A, B)
const int cPWMRes = 8;                    // bit resolution for PWM
const int cMinPWM = 150;                  // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;  // PWM value for maximum speed
const int cPWMFreq = 20000;               // frequency of PWM signal
const int cPotPin = 36;                   // GPIO pin for drive speed potentiometer (A0)
const int cMotorEnablePin = 39;           // GPIO pin for motor enable switch (DIP S1-1)
const int cTrigger = 11;                  // trigger duration in microseconds
const int cMaxEcho = 15000;               // allowable time for valid echo in microseconds
const int cLeftServoPin = 13;             // GPIO pin for left lift servo
const int cRightServoPin = 12;            // GPIO pin for right lift servo
const int cStepPin = 19;                  // GPIO pin for step signal to A4988
const int cDirPin = 18;                   // GPIO pin for direction signal to A4988

//Experiment to determine appropriate values
const int cMotorAdjustment[] = { 0, 0 };    // PWM adjustment for motors to run closer to the same speed
const int cLeftServoHome = 50;              // left servo start angle
const int cLeftServoUp = 130;               // left servo raised angle
const int cRightServoHome = 110;            // right servo start angle
const int cRightServoUp = 32;               // right servo raised angle
const float cLaneSpacingCm = 20.0f;         // chassis width / lane spacing
const float cWheelSpacingCm = 18.5f;        // distance between wheel centers
const float cRefSpeedCmS = 24.0f;           // approximate robot speed at reference PWM
const int cRefPWM = 200;                    // PWM corresponding approximately to cRefSpeedCmS
const float cLaneLengthCm = 390.0f;         // 3.9 m lane length
const uint32_t cUSStopCm = 5;               // stop distance from base station
const uint32_t cUSReadMs = 80;              // ultrasonic read interval during return
const uint32_t cStepperHalfPeriodUs = 600;  // intake stepper speed
const float cGyroDeadzone = 0.005f;         // gyro Z deadzone in rad/s
const float cGyroAlpha = 0.15f;             // gyro Z EMA smoothing

// Variables
boolean motorsEnabled = true;            // motors enabled flag
boolean timeUp3sec = false;              // 3 second timer elapsed flag
boolean timeUp2sec = false;              // 2 second timer elapsed flag
uint32_t lastHeartbeat = 0;              // time of last heartbeat state change
uint32_t curMillis = 0;                  // current time, in milliseconds
uint32_t timerCount3sec = 0;             // 3 second timer count in milliseconds
uint32_t timerCount2sec = 0;             // 2 second timer count in milliseconds
uint32_t robotModeIndex = 0;             // robot operational state
uint32_t lastTime = 0;                   // last time of motor control was updated
Button modeButton = { 0, 0, 0, false };  // NO pushbutton PB1 on GPIO 0, low state when pressed
uint8_t driveSpeed = 0;                  // motor drive speed (0-255)
uint8_t driveIndex = 0;                  // state index for run mode
Ultrasonic ultrasonic = { 21, 22 };      // trigger on GPIO21 and echo on GPIO22
uint32_t lastPulse = 0;                  // last valid pulse duration from HC-SR04
uint32_t cycles = 0;                     // motor cycle count, used to start US measurements
unsigned long mode2StartTime = 0;        // start time for mode 2
uint32_t stateStartMs = 0;               // state start time for mode 2
uint32_t lastPingMs = 0;                 // last ping time for ultrasonic
volatile bool stepperEnabled = false;    // intake stepper enable flag
volatile bool stepPinState = false;      // step pin state
hw_timer_t* pStepperTimer = NULL;        // pointer to timer for stepper
Adafruit_MPU6050 imu;                    // MPU6050 IMU object
AxisFilter gzF;                          // gyro Z filter state
float gbz = 0.0f;                        // gyro Z bias
float yaw = 0.0f;                        // integrated yaw angle in radians
unsigned long lastUsIMU = 0;             // last IMU update time in microseconds
bool imuReady = false;                   // IMU ready flag
bool turnInit = false;                   // per-turn initialization flag

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = { 0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135,
                                        150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0 };

uint32_t modeIndicator[3] = {
  // colours for different modes
  SmartLEDs.Color(255, 0, 0),  //   red - stop
  SmartLEDs.Color(0, 255, 0),  //   green - run
  SmartLEDs.Color(0, 0, 255)   //   blue - full sequence
};

// median-of-3 helper for IMU filtering
static inline float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// deadzone helper for IMU filtering
static inline float deadzone(float x, float d) {
  return (fabs(x) < d) ? 0.0f : x;
}

// exponential moving average helper
static inline float ema(float x, float &y, float alpha) {
  y = (1.0f - alpha) * y + alpha * x;
  return y;
}

// combined filter for gyro axis
static inline float filterAxis(AxisFilter &f, float raw, float dz, float alpha) {
  float m = median3(raw, f.x1, f.x2);
  f.x2 = f.x1;
  f.x1 = raw;
  m = deadzone(m, dz);
  return ema(m, f.y, alpha);
}

// wrap angle to [-pi, pi]
void wrapPi(float &a) {
  while (a > 3.14159265f) a -= 6.28318530f;
  while (a < -3.14159265f) a += 6.28318530f;
}

// initialize IMU
void initIMU() {
  Wire.begin(32, 33);  // SDA on GPIO32, SCL on GPIO33
  delay(100);

  if (!imu.begin()) {
    Serial.println("ERROR: MPU6050 not found");
    imuReady = false;
    return;
  }

  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(200);

  calibrateIMU();
  lastUsIMU = micros();
  imuReady = true;
  Serial.println("MPU6050 found and ready");
}

// calibrate gyro Z bias
void calibrateIMU(uint16_t N, uint16_t dtMs) {
  Serial.println("Calibrating IMU... Keep robot still.");

  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    delay(5);
  }

  double sGZ = 0;
  for (uint16_t i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    sGZ += g.gyro.z;
    delay(dtMs);
  }

  gbz = sGZ / N;

  Serial.print("Gyro Z bias = ");
  Serial.println(gbz, 8);
}

// update integrated yaw angle
void updateIMU() {
  if (!imuReady) return;

  unsigned long nowUs = micros();
  float dt = (nowUs - lastUsIMU) * 1e-6f;
  lastUsIMU = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);

  float gz = g.gyro.z - gbz;
  float fgz = filterAxis(gzF, gz, cGyroDeadzone, cGyroAlpha);

  yaw += fgz * dt;
  wrapPi(yaw);
}

// reset yaw integration to zero
void resetYawDeg() {
  yaw = 0.0f;
  gzF.x1 = 0.0f;
  gzF.x2 = 0.0f;
  gzF.y = 0.0f;
  lastUsIMU = micros();
}

// perform a relative turn using IMU yaw
// turnDir = -1 for left, +1 for right
bool turnToRelativeAngle(float targetDeg, int turnDir) {
  const float RAD2DEG = 57.2957795f;

  if (!imuReady) {
    setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
    setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
    return true;
  }

  if (!turnInit) {
    resetYawDeg();
    turnInit = true;
  }

  updateIMU();

  float yawDeg = yaw * RAD2DEG;
  float progressDeg = (turnDir == -1) ? (yawDeg) : (-yawDeg);

#ifdef DEBUG_STATE
  Serial.printf("Turn target = %.1f deg, yaw = %.2f deg, progress = %.2f deg\n",
                targetDeg, yawDeg, progressDeg);
#endif

  if (progressDeg < targetDeg) {
    if (turnDir == -1) {
      // left turn: both motors same direction
      setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
      setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
    } else {
      // right turn: both motors same direction
      setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
      setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
    }
    return false;
  } else {
    setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
    setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
    turnInit = false;
    return true;
  }
}

void setup() {
  Serial.begin(115200);  // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                     // initialize smart LEDs object
  SmartLEDs.clear();                                     // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0));  // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                            // set brightness [0-255]
  SmartLEDs.show();                                      // update LED

  // setup motors
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);  // setup INT1 GPIO PWM Channel
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);  // setup INT2 GPIO PWM Channel
  }

  // Set up push button
  pinMode(modeButton.pin, INPUT_PULLUP);                                // configure GPIO for mode button pin with internal pullup resistor
  attachInterruptArg(modeButton.pin, buttonISR, &modeButton, FALLING);  // Configure ISR to trigger on low signal on pin

  pinMode(cPotPin, INPUT);          // set up drive speed potentiometer
  pinMode(cMotorEnablePin, INPUT);  // set up motor enable switch (uses external pullup)
  initUltrasonic(&ultrasonic);      // initialize ultrasonic sensor

  // Set up lift servos
  pinMode(cLeftServoPin, OUTPUT);
  pinMode(cRightServoPin, OUTPUT);
  ledcAttach(cLeftServoPin, 50, 14);
  ledcAttach(cRightServoPin, 50, 14);
  setServosHome();

  // Set up stepper driver
  pinMode(cStepPin, OUTPUT);
  pinMode(cDirPin, OUTPUT);
  digitalWrite(cStepPin, LOW);
  digitalWrite(cDirPin, HIGH);

  pStepperTimer = timerBegin(1000000);                       // start timer with 1 MHz frequency
  timerAttachInterrupt(pStepperTimer, &stepperTimerISR);     // configure timer ISR
  timerAlarm(pStepperTimer, cStepperHalfPeriodUs, true, 0);  // set stepper half period

  // Set up IMU
  initIMU();
}

void loop() {
  int pot = 0;             // raw ADC value from pot
  uint32_t pulseDuration;  // duration of pulse read from HC-SR04

  uint32_t curTime = micros();      // capture current time in microseconds
  if (curTime - lastTime > 1000) {  // wait 1 ms
    lastTime = curTime;             // update start time for next control cycle

    // 3 second timer, counts 3000 milliseconds
    timerCount3sec += 1;          // increment 3 second timer count
    if (timerCount3sec > 3000) {  // if 3 seconds have elapsed
      timerCount3sec = 0;         // reset 3 second timer count
      timeUp3sec = true;          // indicate that 3 seconds have elapsed
    }

    // 2 second timer, counts 2000 milliseconds
    timerCount2sec += 1;          // increment 2 second timer count
    if (timerCount2sec > 2000) {  // if 2 seconds have elapsed
      timerCount2sec = 0;         // reset 2 second timer count
      timeUp2sec = true;          // indicate that 2 seconds have elapsed
    }

    if (modeButton.pressed) {               // Change mode on button press
      robotModeIndex++;                     // switch to next mode
      robotModeIndex = robotModeIndex % 3;  // keep mode index between 0 and 2
      if (robotModeIndex == 2) {
        mode2StartTime = millis();
        driveIndex = 0;
        turnInit = false;
      }
      timerCount3sec = 0;          // reset 3 second timer count
      timeUp3sec = false;          // reset 3 second timer
      timerCount2sec = 0;          // reset 2 second timer count
      timeUp2sec = false;          // reset 2 second timer
      modeButton.pressed = false;  // reset flag
    }

    // check if drive motors should be powered
    motorsEnabled = !digitalRead(cMotorEnablePin);  // if SW1-1 is on (low signal), then motors are enabled

    // Read pot to update drive motor speed
    pot = analogRead(cPotPin);
    driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
#ifdef DEBUG_DRIVE_SPEED
    Serial.printf("Drive Speed: Pot R1 = %d, mapped = %d\n", pot, driveSpeed);
#endif

    // modes
    // 0 = Default after power up/reset.           Robot is stopped
    // 1 = Press mode button once to enter.        Run robot
    // 2 = Press mode button twice to enter.       Full sequence
    switch (robotModeIndex) {
      case 0:                                    // Robot stopped
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);  // stop left motor
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);  // stop right motor
        driveIndex = 0;                          // reset drive index
        timerCount2sec = 0;                      // reset 2 second timer count
        timeUp2sec = false;                      // reset 2 second timer
        stepperEnabled = false;                  // stop intake
        setServosHome();                         // reset servos
        turnInit = false;                        // reset IMU turn state
        break;

      case 1:                                            // Run robot
        if (timeUp3sec && motorsEnabled) {               // pause for 3 sec before running case 1 code
                                                         // and run only if enabled
          if (timeUp2sec) {                              // update drive state after 2 seconds
            timerCount2sec = 0;                          // reset 2 second timer count
            timeUp2sec = false;                          // reset 2 second timer
            switch (driveIndex) {                        // cycle through drive states
              case 0:                                    // Stop
                setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);  // stop left motor
                setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);  // stop right motor
                driveIndex = 1;                          // next state: drive forward
                break;

              case 1:  // Drive forward — motors spin in opposite directions as they are opposed by 180 degrees
                // left motor forward, right motor reverse (opposite dir from left)
                setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
                setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
                driveIndex = 2;  // next state: drive backward
                break;

              case 2:  // Drive backward — motors spin in opposite directions as they are opposed by 180 degrees
                // left motor reverse, right motor forward (opposite dir from left)
                setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
                setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
                driveIndex = 3;  // next state: turn left
                break;

              case 3:  // Turn left (counterclockwise) - motors spin in same direction
                // reverse both motors
                setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
                setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
                driveIndex = 4;  // next state: turn right
                break;

              case 4:  // Turn right (clockwise) — motors spin in same direction
                // both motors forward
                setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
                setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
                driveIndex = 0;  // next state: stop
                break;
            }
          }
        } else {                                   // stop when motors are disabled
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);  // stop left motor
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);  // stop right motor
        }
        break;

      case 2:  // Full sequence

        if (!timeUp2sec) {  // wait 2 seconds before starting
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          break;
        }

        if (motorsEnabled) {  // and run only if enabled

          const uint32_t longPassTimeMs = 1200;  // straight timing
          const uint32_t shiftTimeMs = 1200;     // shift timing to switch lanes
          const float turn90Deg = 78.0f;         // IMU turn target to cover 90 degree turns (tolerance included here)

          switch (driveIndex) {

            case 0:                     // Start sequence
              setServosHome();          // set servos to start position
              stepperEnabled = true;    // intake on
              cycles = 0;               // reset ultrasonic pacing
              lastPulse = 0;            // clear ultrasonic reading
              stateStartMs = millis();  // start timing
              turnInit = false;         // reset turn initialization
              driveIndex = 1;           // next state: lane 1 up
              break;

            case 1:  // Lane 1: drive up
              setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
              setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);

              if (millis() - stateStartMs >= longPassTimeMs) {  // reached end of lane 1
                setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
                stateStartMs = millis();
                turnInit = false;
                driveIndex = 2;  // next state: left turn
              }
              break;

            case 2:  // Turn left 90 degrees using IMU
              if (turnToRelativeAngle(turn90Deg, -1)) {
                stateStartMs = millis();
                driveIndex = 3;  // next state: shift lane
              }
              break;

            case 3:  // Shift left to next lane
              setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
              setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);

              if (millis() - stateStartMs >= shiftTimeMs) {  // finished shift
                setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
                stateStartMs = millis();
                turnInit = false;
                driveIndex = 4;  // next state: left turn
              }
              break;

            case 4:  // Turn left 90 degrees using IMU
              if (turnToRelativeAngle(turn90Deg, -1)) {
                stateStartMs = millis();
                driveIndex = 5;  // next state: lane 2 down
              }
              break;

            case 5:  // Lane 2: drive down
              setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
              setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);

              if (millis() - stateStartMs >= longPassTimeMs) {  // reached end of lane 2
                setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
                stepperEnabled = false;  // intake off at end of lane 2
                cycles = 0;
                lastPulse = 0;
                turnInit = false;
                driveIndex = 6;  // next state: right turn
              }
              break;

            case 6:  // Turn right 90 degrees using IMU
              if (turnToRelativeAngle(turn90Deg, 1)) {
                setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
                cycles = 0;
                lastPulse = 0;
                turnInit = false;
                driveIndex = 7;  // next state: reverse to base
              }
              break;

            case 7: {  // Reverse to base using rear ultrasonic
              setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
              setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);

              if (cycles == 0) ping(&ultrasonic);
              cycles++;
              if (cycles >= 100) cycles = 0;

              pulseDuration = getEchoTime(&ultrasonic);
              if (pulseDuration > 0) {
                lastPulse = pulseDuration;
#ifdef DEBUG_ULTRASONIC
                Serial.printf("Rear ultrasonic: %ld us = %ld cm = %ld in\n",
                              lastPulse, usToCm(lastPulse), usToIn(lastPulse));
#endif
                if (usToCm(lastPulse) <= cUSStopCm) {
                  setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
                  setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
                  turnInit = false;
                  driveIndex = 8;  // next state: final right turn
                }
              }
              break;
            }

            case 8:  // Turn right 90 degrees using IMU
              if (turnToRelativeAngle(turn90Deg, 1)) {
                setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
                setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
                stateStartMs = millis();
                driveIndex = 9;  // next state: lift servos
              }
              break;

            case 9:  // Lift servos
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
              setServosUp();
              stateStartMs = millis();
              driveIndex = 10;
              break;

            case 10:  // Finished
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
              stepperEnabled = false;
              robotModeIndex = 0;
              driveIndex = 0;
              break;
          }
        } else {  // motors disabled
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          stepperEnabled = false;
          turnInit = false;
        }
        break;
    }
  }

  doHeartbeat();
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;
    LEDBrightnessIndex++;
    if (LEDBrightnessIndex >= sizeof(LEDBrightnessLevels)) {
      LEDBrightnessIndex = 0;
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);
    SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);
    SmartLEDs.show();
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {  // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  } else if (dir == -1) {  // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  } else {  // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// button interrupt service routine
// argument is pointer to button structure, which is statically cast to a Button structure,
// allowing multiple instances of the buttonISR to be created (1 per button)
void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);

  uint32_t pressTime = millis();
  if (pressTime > s->nextPressTime) {
    s->numberPresses += 1;
    s->pressed = true;
    s->nextPressTime = pressTime + cDebounceDelay;
  }
}

// encoder interrupt service routine
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
}

// initialize ultrasonic sensor I/O and associated interrupts
void initUltrasonic(Ultrasonic* us) {
  pinMode(us->triggerPin, OUTPUT);
  pinMode(us->echoPin, INPUT);
  attachInterruptArg(us->echoPin, echoISR, us, CHANGE);
  us->pTimer = timerBegin(1000000);
  timerAttachInterruptArg(us->pTimer, usTimerISR, us);
}

// start ultrasonic echo measurement by sending pulse to trigger pin
void ping(Ultrasonic* us) {
  timerRestart(us->pTimer);
  timerAlarm(us->pTimer, cTrigger, false, 0);
  digitalWrite(us->triggerPin, HIGH);
  us->timeout = false;
}

// if new pulse received on echo pin calculate echo duration in microseconds
uint32_t getEchoTime(Ultrasonic* us) {
  if (us->newEcho && !us->timeout) {
    us->newEcho = false;
    return us->pulseEnd - us->pulseBegin;
  } else {
    return 0;
  }
}

// convert echo time in microseconds to centimetres
uint32_t usToCm(uint32_t us) {
  return (uint32_t)(double)us * 0.01724;
}

// convert echo time in microseconds to inches
uint32_t usToIn(uint32_t us) {
  return (uint32_t)(double)us * 0.006757;
}

// convert servo angle to duty
uint32_t angleToDuty(int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t us = map(angle, 0, 180, 500, 2500);
  return (uint32_t)((us / 20000.0f) * 16383.0f);
}

void setServosHome() {
  ledcWrite(cLeftServoPin, angleToDuty(cLeftServoHome));
  ledcWrite(cRightServoPin, angleToDuty(cRightServoHome));
}

void setServosUp() {
  ledcWrite(cLeftServoPin, angleToDuty(cLeftServoUp));
  ledcWrite(cRightServoPin, angleToDuty(cRightServoUp));
}

// timer interrupt service routine
void ARDUINO_ISR_ATTR usTimerISR(void* arg) {
  Ultrasonic* us = static_cast<Ultrasonic*>(arg);

  if (timerRead(us->pTimer) < cTrigger + 10) {
    digitalWrite(us->triggerPin, LOW);
    timerRestart(us->pTimer);
    timerAlarm(us->pTimer, cMaxEcho, false, 0);
  } else {
    us->timeout = true;
  }
}

// echo interrupt service routine
void ARDUINO_ISR_ATTR echoISR(void* arg) {
  Ultrasonic* us = static_cast<Ultrasonic*>(arg);

  if (digitalRead(us->echoPin)) {
    us->pulseBegin = micros();
  } else if (!us->timeout) {
    us->pulseEnd = micros();
    us->newEcho = true;
  }
}

// timer interrupt service routine for stepper motor
void ARDUINO_ISR_ATTR stepperTimerISR() {
  if (stepperEnabled) {
    stepPinState = !stepPinState;
    digitalWrite(cStepPin, stepPinState);
  } else {
    stepPinState = false;
    digitalWrite(cStepPin, LOW);
  }
}