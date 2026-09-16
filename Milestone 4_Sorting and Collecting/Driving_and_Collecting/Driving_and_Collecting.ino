#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void setIntakeMotor(int dir, int pwm);
void initUltrasonic(struct Ultrasonic* us);
void ping(struct Ultrasonic* us);
uint32_t getEchoTime(struct Ultrasonic* us);
uint32_t usToCm(uint32_t us);
uint32_t usToIn(uint32_t us);
uint32_t angleToDuty(int angle);
void writeServos(int leftAngle, int rightAngle);
void setServosHome();
void ARDUINO_ISR_ATTR usTimerISR(void* arg);
void ARDUINO_ISR_ATTR echoISR(void* arg);

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

// Constants
const int cHeartbeatInterval = 75;        // heartbeat update interval, in milliseconds
const int cSmartLED = 23;                 // SMART LED connected to GPIO23
const int cSmartLEDCount = 1;             // number of Smart LEDs in use

const int cIN1Pin[] = { 26, 16 };         // GPIO pin(s) for IN1 for left and right motors (A, B)
const int cIN2Pin[] = { 27, 17 };         // GPIO pin(s) for IN2 for left and right motors (A, B)
const int cPWMFreq = 20000;               // frequency of PWM signal
const int cPWMRes = 8;                    // bit resolution for PWM
const int cPotPin = 36;                   // GPIO pin for drive speed potentiometer (A0)
const int cMinPWM = 150;                  // PWM value for minimum speed that turns motor
const int cMaxPWM = 255;                  // PWM value for maximum speed

const int cLeftServoPin = 13;             // GPIO pin for left lift servo
const int cRightServoPin = 12;            // GPIO pin for right lift servo
const int cLeftServoHome = 50;            // left servo start angle
const int cRightServoHome = 110;          // right servo start angle

const int cIntakeIN1 = 18;                // GPIO pin for intake motor driver IN1
const int cIntakeIN2 = 19;                // GPIO pin for intake motor driver IN2
const int cIntakePWM = 200;               // intake motor speed

const int cTrigger = 11;                  // trigger duration in microseconds
const int cMaxEcho = 15000;               // allowable time for valid echo in microseconds
const uint32_t cUSStopCm = 1;             // stop distance from base station

const int cMotorAdjustment[] = { 0, 0 };  // PWM adjustment for motors to run closer to the same speed
const uint32_t cForwardTimeMs = 3000;     // forward drive time
const uint32_t cServoStepMs = 25;         // servo step time
const int cServoSteps = 120;              // servo sweep steps

// Variables
boolean timeUp3sec = false;             // 3 second timer elapsed flag
boolean timeUp2sec = false;             // 2 second timer elapsed flag
uint32_t timerCount3sec = 0;            // 3 second timer count in milliseconds
uint32_t timerCount2sec = 0;            // 2 second timer count in milliseconds
uint32_t lastTime = 0;                  // last time of control was updated
uint32_t lastHeartbeat = 0;             // time of last heartbeat state change

uint8_t driveSpeed = 0;                 // motor drive speed (0-255)
uint8_t driveIndex = 0;                 // state index
uint8_t robotModeIndex = 2;             // 0 = stopped, 2 = sequence active

Ultrasonic ultrasonic = { 21, 22 };     // trigger on GPIO21 and echo on GPIO22
uint32_t lastPulse = 0;                 // last valid pulse duration from HC-SR04
uint32_t cycles = 0;                    // ultrasonic pacing counter
uint32_t stateStartMs = 0;              // state start time
uint32_t servoStepStartMs = 0;          // servo step start time
int servoStepIndex = 0;                 // servo sweep index

// Smart LED
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = { 0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135,
                                        150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0 };

uint32_t modeIndicator[3] = {
  SmartLEDs.Color(255, 0, 0),  // red - stopped
  SmartLEDs.Color(0, 255, 0),  // green
  SmartLEDs.Color(0, 0, 255)   // blue - sequence
};

void setup() {
  Serial.begin(115200);

  SmartLEDs.begin();
  SmartLEDs.clear();
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0));
  SmartLEDs.setBrightness(0);
  SmartLEDs.show();

  for (int k = 0; k < 2; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);
  }

  ledcAttach(cIntakeIN1, cPWMFreq, cPWMRes);
  ledcAttach(cIntakeIN2, cPWMFreq, cPWMRes);

  pinMode(cLeftServoPin, OUTPUT);
  pinMode(cRightServoPin, OUTPUT);
  ledcAttach(cLeftServoPin, 50, 14);
  ledcAttach(cRightServoPin, 50, 14);

  pinMode(cPotPin, INPUT);

  initUltrasonic(&ultrasonic);

  setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
  setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
  setIntakeMotor(0, 0);
  setServosHome();
}

void loop() {
  int pot = 0;
  uint32_t pulseDuration;

  uint32_t curTime = micros();
  if (curTime - lastTime > 1000) {
    lastTime = curTime;

    timerCount3sec += 1;
    if (timerCount3sec > 3000) {
      timerCount3sec = 0;
      timeUp3sec = true;
    }

    timerCount2sec += 1;
    if (timerCount2sec > 2000) {
      timerCount2sec = 0;
      timeUp2sec = true;
    }

    pot = analogRead(cPotPin);
    driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);

    switch (driveIndex) {
      case 0:  // Wait 2 seconds before starting
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);
        setServosHome();

        if (timeUp2sec) {
          timerCount2sec = 0;
          timeUp2sec = false;
          stateStartMs = millis();
          driveIndex = 1;
        }
        break;

      case 1:  // Drive forward with intake on
        setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - stateStartMs >= cForwardTimeMs) {
          cycles = 0;
          lastPulse = 0;
          driveIndex = 2;
        }
        break;

      case 2:  // Reverse to base using rear ultrasonic with intake on
        setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (cycles == 0) ping(&ultrasonic);
        cycles++;
        if (cycles >= 100) cycles = 0;

        pulseDuration = getEchoTime(&ultrasonic);
        if (pulseDuration > 0) {
          lastPulse = pulseDuration;
          if (usToCm(lastPulse) <= cUSStopCm) {
            setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
            setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
            servoStepStartMs = millis();
            servoStepIndex = 0;
            driveIndex = 3;
          }
        }
        break;

      case 3:  // Lift servos immediately
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex <= cServoSteps) {
            int leftAngle = cLeftServoHome + servoStepIndex;
            int rightAngle = cRightServoHome - servoStepIndex;
            writeServos(leftAngle, rightAngle);
            servoStepIndex++;
          } else {
            timerCount3sec = 0;
            timeUp3sec = false;
            driveIndex = 4;
          }
        }
        break;

      case 4:  // Hold servos up for 3 seconds
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (timeUp3sec) {
          timerCount3sec = 0;
          timeUp3sec = false;
          servoStepStartMs = millis();
          servoStepIndex = cServoSteps;
          driveIndex = 5;
        }
        break;

      case 5:  // Lower servos
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex >= 0) {
            int leftAngle = cLeftServoHome + servoStepIndex;
            int rightAngle = cRightServoHome - servoStepIndex;
            writeServos(leftAngle, rightAngle);
            servoStepIndex--;
          } else {
            driveIndex = 6;
            robotModeIndex = 0;
          }
        }
        break;

      case 6:  // Finished
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);
        setServosHome();
        break;
    }
  }

  doHeartbeat();
}

void doHeartbeat() {
  uint32_t curMillis = millis();
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

void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  } else if (dir == -1) {
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  } else {
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

void setIntakeMotor(int dir, int pwm) {
  if (dir == 1) {
    ledcWrite(cIntakeIN1, pwm);
    ledcWrite(cIntakeIN2, 0);
  } else if (dir == -1) {
    ledcWrite(cIntakeIN1, 0);
    ledcWrite(cIntakeIN2, pwm);
  } else {
    ledcWrite(cIntakeIN1, 0);
    ledcWrite(cIntakeIN2, 0);
  }
}

void initUltrasonic(Ultrasonic* us) {
  pinMode(us->triggerPin, OUTPUT);
  pinMode(us->echoPin, INPUT);
  attachInterruptArg(us->echoPin, echoISR, us, CHANGE);
  us->pTimer = timerBegin(1000000);
  timerAttachInterruptArg(us->pTimer, usTimerISR, us);
}

void ping(Ultrasonic* us) {
  timerRestart(us->pTimer);
  timerAlarm(us->pTimer, cTrigger, false, 0);
  digitalWrite(us->triggerPin, HIGH);
  us->timeout = false;
}

uint32_t getEchoTime(Ultrasonic* us) {
  if (us->newEcho && !us->timeout) {
    us->newEcho = false;
    return us->pulseEnd - us->pulseBegin;
  } else {
    return 0;
  }
}

uint32_t usToCm(uint32_t us) {
  return (uint32_t)(double)us * 0.01724;
}

uint32_t usToIn(uint32_t us) {
  return (uint32_t)(double)us * 0.006757;
}

uint32_t angleToDuty(int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t us = map(angle, 0, 180, 500, 2500);
  return (uint32_t)((us / 20000.0f) * 16383.0f);
}

void writeServos(int leftAngle, int rightAngle) {
  ledcWrite(cLeftServoPin, angleToDuty(leftAngle));
  ledcWrite(cRightServoPin, angleToDuty(rightAngle));
}

void setServosHome() {
  ledcWrite(cLeftServoPin, angleToDuty(cLeftServoHome));
  ledcWrite(cRightServoPin, angleToDuty(cRightServoHome));
}

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

void ARDUINO_ISR_ATTR echoISR(void* arg) {
  Ultrasonic* us = static_cast<Ultrasonic*>(arg);

  if (digitalRead(us->echoPin)) {
    us->pulseBegin = micros();
  } else if (!us->timeout) {
    us->pulseEnd = micros();
    us->newEcho = true;
  }
}