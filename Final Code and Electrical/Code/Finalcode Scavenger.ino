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
  const int triggerPin;
  const int echoPin;
  volatile uint32_t pulseBegin = 0;
  volatile uint32_t pulseEnd = 0;
  volatile bool newEcho = false;
  volatile bool timeout = false;
  hw_timer_t* pTimer = NULL;
};

// Constants
const int cHeartbeatInterval = 75;
const int cSmartLED = 23;
const int cSmartLEDCount = 1;

const int cIN1Pin[] = { 26, 16 };
const int cIN2Pin[] = { 27, 17 };
const int cPWMFreq = 20000;
const int cPWMRes = 8;
const int cPotPin = 36;
const int cMinPWM = 150;
const int cMaxPWM = 255;

const int cLeftServoPin = 13;
const int cRightServoPin = 12;
const int cLeftServoHome = 50;    // Left Servo's Initial Position At 0 Degrees On The Chassis
const int cRightServoHome = 110;  // Right Servo's Initial Position At 0 Degrees On The Chassis

const int cIntakeIN1 = 18;   // IN1 Pin For Intake Motor's Motor Driver
const int cIntakeIN2 = 19;   // IN2 Pin For Intake Motor's Motor Driver
const int cIntakePWM = 250;  // Speed Of The Intake Motor

const int cTrigger = 11;
const int cMaxEcho = 15000;
const uint32_t cUSStopCm = 15;  // Distance The Ultrasonic Reads To Stop

const int cMotorAdjustment[] = { 0, -5 };  // Motor Adjustment For More Equal And Straight Driving
const uint32_t cForwardTimeMs = 4000;      // Forward Drive Time, Tuned To Cover The Collection Path
const uint32_t cTurnTimeMs = 1400;         // Turn Time Tuned For The Angled Collection Paths
const uint32_t cServoStepMs = 25;
const int cServoSteps = 110;  // Tuned To Get To The Right Angle When Scoop Is Lifted To The Sorter Hopper

const int cMidLiftDeg = 50;                  // Midlift To Keep The Erasers From Falling When Reversing
const uint32_t cPostStopAlignMs = 5000;     // Additional Driving Delay For The Scavenger To Align With The Base Station Using The Guiding Arms/Rails

// Variables
boolean timeUp6sec = false;
boolean timeUp2sec = false;
uint32_t timerCount6sec = 0;
uint32_t timerCount2sec = 0;
uint32_t lastTime = 0;
uint32_t lastHeartbeat = 0;

uint8_t driveSpeed = 0;
uint8_t driveIndex = 0;
uint8_t robotModeIndex = 2;

Ultrasonic ultrasonic = { 21, 22 };
uint32_t lastPulse = 0;
uint32_t cycles = 0;
uint32_t stateStartMs = 0;
uint32_t servoStepStartMs = 0;
int servoStepIndex = 0;

// Smart LED
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = { 0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135,
                                        150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0 };

uint32_t modeIndicator[3] = {
  SmartLEDs.Color(255, 0, 0),
  SmartLEDs.Color(0, 255, 0),
  SmartLEDs.Color(0, 0, 255)
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

    timerCount6sec += 1;
    if (timerCount6sec > 6000) {
      timerCount6sec = 0;
      timeUp6sec = true;
    }

    timerCount2sec += 1;
    if (timerCount2sec > 2000) {
      timerCount2sec = 0;
      timeUp2sec = true;
    }

    pot = analogRead(cPotPin);
    driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);

    switch (driveIndex) {
      case 0:  // Wait 2 Seconds Before Starting
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

        // Cycle 1: FRONT SEQUENCE

      case 1:  // Drive Forward With Intake On
        setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - stateStartMs >= cForwardTimeMs) {
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          setIntakeMotor(-1, cIntakePWM);
          servoStepStartMs = millis();
          servoStepIndex = 0;
          driveIndex = 2;
        }
        break;

      case 2:  // Pre-Lift To Stop Erasers From Falling Back Out
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex <= cMidLiftDeg) {
            writeServos(cLeftServoHome + servoStepIndex, cRightServoHome - servoStepIndex);
            servoStepIndex++;
          } else {
            cycles = 0;
            lastPulse = 0;
            driveIndex = 3;
          }
        }
        break;

      case 3:  // Reverse Straight To Base
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
            stateStartMs = millis();
            driveIndex = 4;
          }
        }
        break;

      case 4:  // Post-Stop Align To Make Sure The Scavenger Is Aligned With Base Before Drop-Off
        setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - stateStartMs >= cPostStopAlignMs) {
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          servoStepStartMs = millis();
          servoStepIndex = cMidLiftDeg;
          driveIndex = 5;
        }
        break;

      case 5:  // Lift Rest Of Way For Drop-Off
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex <= cServoSteps) {
            writeServos(cLeftServoHome + servoStepIndex, cRightServoHome - servoStepIndex);
            servoStepIndex++;
          } else {
            timerCount6sec = 0;
            timeUp6sec = false;
            driveIndex = 6;
          }
        }
        break;

      case 6:  // Hold Up For 6 Seconds To Make Sure The Scoop Is Empty Before Moving Off
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (timeUp6sec) {
          timerCount6sec = 0;
          timeUp6sec = false;
          servoStepStartMs = millis();
          servoStepIndex = cServoSteps;
          driveIndex = 7;
        }
        break;

      case 7:  // Lower Servos Back To 0 Degrees Relative To The Chassis
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex >= 0) {
            writeServos(cLeftServoHome + servoStepIndex, cRightServoHome - servoStepIndex);
            servoStepIndex--;
          } else {
            setServosHome();
            stateStartMs = millis();
            driveIndex = 8;
          }
        }
        break;

      case 8:  // Turn Right In Place At Base
        setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (millis() - stateStartMs >= cTurnTimeMs) {
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          stateStartMs = millis();
          driveIndex = 9;
        }
        break;

        // Cycle 2: RIGHT SIDE

      case 9:  // Drive Forward Straight
        setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - stateStartMs >= cForwardTimeMs) {
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          setIntakeMotor(-1, cIntakePWM);
          servoStepStartMs = millis();
          servoStepIndex = 0;
          driveIndex = 10;
        }
        break;

      case 10:  // Pre-Lift To Stop Erasers From Falling Back Out
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex <= cMidLiftDeg) {
            writeServos(cLeftServoHome + servoStepIndex, cRightServoHome - servoStepIndex);
            servoStepIndex++;
          } else {
            cycles = 0;
            lastPulse = 0;
            driveIndex = 11;
          }
        }
        break;

      case 11:  // Reverse Straight Back To The Base
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
            stateStartMs = millis();
            driveIndex = 12;
          }
        }
        break;

      case 12:  // Turn Left In Place To Re-Align
        setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - stateStartMs >= cTurnTimeMs - 200) {
          stateStartMs = millis();
          driveIndex = 13;
        }
        break;

      case 13:  // Post-Stop Align To Make Sure The Scavenger Is Aligned With Base Before Drop-Off
        setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - stateStartMs >= cPostStopAlignMs) {
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          servoStepStartMs = millis();
          servoStepIndex = cMidLiftDeg;
          driveIndex = 14;
        }
        break;

      case 14:  // Lift Rest Of Way For Drop-Off
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex <= cServoSteps) {
            writeServos(cLeftServoHome + servoStepIndex, cRightServoHome - servoStepIndex);
            servoStepIndex++;
          } else {
            timerCount6sec = 0;
            timeUp6sec = false;
            driveIndex = 15;
          }
        }
        break;

      case 15:  // Hold Up For 6 Seconds To Make Sure The Scoop Is Empty Before Lowering
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (timeUp6sec) {
          timerCount6sec = 0;
          timeUp6sec = false;
          servoStepStartMs = millis();
          servoStepIndex = cServoSteps;
          driveIndex = 16;
        }
        break;

      case 16:  // Lower Servos Back To 0 Degrees Relative To The Chassis
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex >= 0) {
            writeServos(cLeftServoHome + servoStepIndex, cRightServoHome - servoStepIndex);
            servoStepIndex--;
          } else {
            setServosHome();
            stateStartMs = millis();
            driveIndex = 17;
          }
        }
        break;

      case 17:  // Turn Left In Place At Base
        setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (millis() - stateStartMs >= (cTurnTimeMs + 500)) {
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          stateStartMs = millis();
          driveIndex = 18;
        }
        break;

        // Cycle 3: LEFT SIDE

      case 18:  // Drive Forward Straight With The Intake On
        setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - stateStartMs >= cForwardTimeMs) {
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          setIntakeMotor(-1, cIntakePWM);
          servoStepStartMs = millis();
          servoStepIndex = 0;
          driveIndex = 19;
        }
        break;

      case 19:  // Pre-Lift To Stop Erasers From Falling Back Out
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex <= cMidLiftDeg) {
            writeServos(cLeftServoHome + servoStepIndex, cRightServoHome - servoStepIndex);
            servoStepIndex++;
          } else {
            cycles = 0;
            lastPulse = 0;
            driveIndex = 20;
          }
        }
        break;

      case 20:  // Reverse Straight Back To The Base
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
            stateStartMs = millis();
            driveIndex = 21;
          }
        }
        break;

      case 21:  // Turn Right In Place To Re-Align
        setMotor(-1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - stateStartMs >= cTurnTimeMs) {
          stateStartMs = millis();
          driveIndex = 22;
        }
        break;

      case 22:  // Post-Stop Align To Make Sure The Scavenger Is Aligned With Base Before Drop-Off
        setMotor(1, driveSpeed + cMotorAdjustment[0], cIN1Pin[0], cIN2Pin[0]);
        setMotor(-1, driveSpeed + cMotorAdjustment[1], cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - stateStartMs >= cPostStopAlignMs) {
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
          servoStepStartMs = millis();
          servoStepIndex = cMidLiftDeg;
          driveIndex = 23;
        }
        break;

      case 23:  // Lift Rest Of Way For Drop-Off
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(-1, cIntakePWM);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex <= cServoSteps) {
            writeServos(cLeftServoHome + servoStepIndex, cRightServoHome - servoStepIndex);
            servoStepIndex++;
          } else {
            timerCount6sec = 0;
            timeUp6sec = false;
            driveIndex = 24;
          }
        }
        break;

      case 24:  // Hold Up For 6 Sec To Make Sure The Scoop Is Empty Before Lowering
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (timeUp6sec) {
          timerCount6sec = 0;
          timeUp6sec = false;
          servoStepStartMs = millis();
          servoStepIndex = cServoSteps;
          driveIndex = 25;
        }
        break;

      case 25:  // Lower Servos Back To 0 Degrees Relative To The Chassis And Finish
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);
        setIntakeMotor(0, 0);

        if (millis() - servoStepStartMs >= cServoStepMs) {
          servoStepStartMs = millis();
          if (servoStepIndex >= 0) {
            writeServos(cLeftServoHome + servoStepIndex, cRightServoHome - servoStepIndex);
            servoStepIndex--;
          } else {
            driveIndex = 26;
            robotModeIndex = 0;
          }
        }
        break;

      case 26:  // Finished
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