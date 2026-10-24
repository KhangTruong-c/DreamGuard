
#include <Wire.h>
#include <LiquidCrystal.h>
#include <math.h>
#include "MAX30105.h"
#include "heartRate.h"

// DreamGuard
// Arduino UNO
// LCD: RS=12, E=11, D4=10, D5=9, D6=8, D7=7
// Motor driver input: D3
// Reset button: D2 to GND
// I2C: A4 SDA, A5 SCL

LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

const byte MOTOR_PIN = 3;
const byte BUTTON_PIN = 2;
const byte MPU_ADDR = 0x68;

// Timings
const unsigned long STAGE1_MS = 5000UL;
const unsigned long COOLDOWN_MS = 15000UL;
const unsigned long OBSERVE_MS = 15000UL;
const unsigned long TRIGGER_MS = 3000UL;

// Trigger thresholds
const float MOVEMENT_THRESHOLD = 0.45f;
const float HR_TRIGGER_MULTIPLIER = 1.40f;

// MAX30102
MAX30105 heartSensor;
bool maxOK = false;
bool haveBeat = false;
bool finger = false;

const uint32_t FINGER_IR_MIN = 50000;
uint32_t lastBeat = 0;
uint32_t lastIR = 0;

byte rateIndex = 0;
byte rateCount = 0;
byte rates[4] = {0, 0, 0, 0};

// Modes
enum Mode {
  CALIBRATING,
  RESTING,
  SLEEPING,
  STAGE1,
  COOLDOWN,
  MONITORING,
  STAGE2,
  NO_SIGNAL
};

Mode mode = CALIBRATING;

// Timers
unsigned long modeStart = 0;
unsigned long elevatedSince = 0;
unsigned long lastMotion = 0;
unsigned long lastDisplay = 0;
unsigned long lastLog = 0;
unsigned long lastButton = 0;

bool elevatedTracking = false;
bool mpuOK = false;

// Motion
float motion = 0;
float lastX = 0;
float lastY = 0;
float lastZ = 0;
bool previousMotion = false;

// Heart rate
int restingHR = 0;
int currentHR = 0;

bool restingHRReady = false;
bool hrValid = false;
bool motorOn = false;

// --------------------------------------------------
// 3-SECOND ROLLING MOVEMENT WINDOW
// --------------------------------------------------

// MPU readings occur every 50 ms (20 Hz).
// 60 readings = approximately 3 seconds.
// 54/60 readings must exceed 0.45.

const byte WINDOW_SAMPLES = 60;
const byte REQUIRED_HIGH_SAMPLES = 54;

byte movementHistory[WINDOW_SAMPLES] = {0};

byte historyIndex = 0;
byte historyCount = 0;
byte highCount = 0;

unsigned long lowSince = 0;

void resetMovementWindow() {
  historyIndex = 0;
  historyCount = 0;
  highCount = 0;
  lowSince = 0;

  for (byte i = 0; i < WINDOW_SAMPLES; i++) {
    movementHistory[i] = 0;
  }
}

void recordMovementSample() {
  byte high = (mpuOK && motion > MOVEMENT_THRESHOLD) ? 1 : 0;

  if (historyCount == WINDOW_SAMPLES) {
    highCount -= movementHistory[historyIndex];
  } else {
    historyCount++;
  }

  movementHistory[historyIndex] = high;
  highCount += high;

  historyIndex = (historyIndex + 1) % WINDOW_SAMPLES;
}

bool movementWindowTriggered() {
  return historyCount == WINDOW_SAMPLES &&
         highCount >= REQUIRED_HIGH_SAMPLES;
}

// --------------------------------------------------
// MODE NAMES
// --------------------------------------------------

const char *modeName(Mode m) {
  switch (m) {
    case CALIBRATING: return "CALIBRATING";
    case RESTING:     return "RESTING";
    case SLEEPING:    return "SLEEPING";
    case STAGE1:      return "STAGE 1";
    case COOLDOWN:    return "COOLDOWN";
    case MONITORING:  return "MONITORING";
    case STAGE2:      return "STAGE 2";
    default:          return "NO HR SIGNAL";
  }
}

// --------------------------------------------------
// MOTOR
// --------------------------------------------------

void motor(byte power) {
  analogWrite(MOTOR_PIN, power);
  motorOn = power > 0;
}

// --------------------------------------------------
// MODE TRANSITIONS
// --------------------------------------------------

void enter(Mode next) {
  mode = next;
  modeStart = millis();

  elevatedTracking = false;

  if (next == MONITORING) {
    resetMovementWindow();
  }

  motor(0);

  Serial.print(F("EVENT t="));
  Serial.print(modeStart);
  Serial.print(F(" MODE="));
  Serial.println(modeName(next));
}

// --------------------------------------------------
// LCD DISPLAY
// --------------------------------------------------

void displayStatus() {
  lcd.setCursor(0, 0);
  lcd.print(F("HR:             "));
  lcd.setCursor(4, 0);

  if (hrValid) {
    lcd.print(currentHR);
    lcd.print(F(" BPM"));
  } else {
    lcd.print(F("NO SIGNAL"));
  }

  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);

  lcd.print(F("MODE: "));
  lcd.print(modeName(mode));
}

// --------------------------------------------------
// RESET BUTTON
// --------------------------------------------------

bool buttonPressed() {
  static bool oldState = HIGH;

  bool newState = digitalRead(BUTTON_PIN);

  bool pressed =
    (oldState == HIGH && newState == LOW &&
     millis() - lastButton >= 50);

  if (newState != oldState) {
    lastButton = millis();
  }

  oldState = newState;

  return pressed;
}

// --------------------------------------------------
// SERIAL LOGGING
// --------------------------------------------------

void outputLog() {
  Serial.print(F("t="));
  Serial.print(millis());

  Serial.print(F(" MODE="));
  Serial.print(modeName(mode));

  Serial.print(F(" HR="));

  if (hrValid) {
    Serial.print(currentHR);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" REST="));

  if (restingHRReady) {
    Serial.print(restingHR);
  } else {
    Serial.print(F("UNCALIBRATED"));
  }

  Serial.print(F(" HR_LIMIT="));

  if (restingHRReady) {
    Serial.print(restingHR * HR_TRIGGER_MULTIPLIER, 1);
  } else {
    Serial.print(F("N/A"));
  }

  Serial.print(F(" MOVE="));
  Serial.print(motion, 2);

  // Rolling-window debugging
  Serial.print(F(" WINDOW="));
  Serial.print(highCount);
  Serial.print('/');
  Serial.print(historyCount);

  Serial.print(F(" READY="));
  Serial.print(movementWindowTriggered() ? 1 : 0);

  Serial.print(F(" MPU="));
  Serial.print(mpuOK ? 1 : 0);

  Serial.print(F(" MOTOR="));
  Serial.println(motorOn ? 1 : 0);
}

// --------------------------------------------------
// MPU6050
// --------------------------------------------------

bool mpuWrite(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool accel(float &x, float &y, float &z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write((byte)0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU_ADDR, (byte)6) != (byte)6) {
    return false;
  }

  int16_t a[3];

  for (byte i = 0; i < 3; i++) {
    uint16_t hi = (uint8_t)Wire.read();
    uint16_t lo = (uint8_t)Wire.read();

    a[i] = (int16_t)((hi << 8) | lo);
  }

  x = a[0] * (9.80665f / 16384.0f);
  y = a[1] * (9.80665f / 16384.0f);
  z = a[2] * (9.80665f / 16384.0f);

  return true;
}

// --------------------------------------------------
// REAL MOVEMENT READINGS
// --------------------------------------------------

void realMotion() {
  float x, y, z;

  mpuOK = accel(x, y, z);

  if (!mpuOK) {
    motion = 0;
    previousMotion = false;
    return;
  }

  if (previousMotion) {
    float dx = x - lastX;
    float dy = y - lastY;
    float dz = z - lastZ;

    motion = sqrt(dx * dx + dy * dy + dz * dz);
  } else {
    motion = 0;
    previousMotion = true;
  }

  lastX = x;
  lastY = y;
  lastZ = z;
}

// --------------------------------------------------
// HEART RATE
// --------------------------------------------------

void resetPulse() {
  haveBeat = false;
  rateCount = 0;
  rateIndex = 0;

  hrValid = false;
  currentHR = 0;
}

void startMax() {
  maxOK = heartSensor.begin(
    Wire,
    I2C_SPEED_STANDARD,
    0x57
  );

  if (!maxOK) {
    Serial.println(F("ERROR MAX30102_INIT=0"));
    return;
  }

  heartSensor.setup(
    0x1F,
    4,
    2,
    400,
    411,
    4096
  );

  heartSensor.setPulseAmplitudeRed(0x1F);
  heartSensor.setPulseAmplitudeIR(0x1F);
  heartSensor.setPulseAmplitudeGreen(0);

  heartSensor.clearFIFO();

  lastIR = millis();

  Serial.println(F("MAX30102_INIT=1"));
}

// --------------------------------------------------
// READ MAX30102
// --------------------------------------------------

void readMax() {
  if (!maxOK) {
    resetPulse();
    return;
  }

  heartSensor.check();

  while (heartSensor.available()) {
    long ir = (long)heartSensor.getFIFOIR();

    heartSensor.nextSample();
    lastIR = millis();

    finger = ir > FINGER_IR_MIN;

    bool beat = checkForBeat(ir);

    if (!finger) {
      resetPulse();
      continue;
    }

    if (beat) {
      unsigned long time = millis();

      if (!haveBeat) {
        lastBeat = time;
        haveBeat = true;
        continue;
      }

      unsigned long delta = time - lastBeat;

      if (delta < 300) {
        continue;
      }

      lastBeat = time;

      if (delta > 1500) {
        resetPulse();
        continue;
      }

      byte bpm = (byte)(60000UL / delta);

      rates[rateIndex] = bpm;
      rateIndex = (rateIndex + 1) % 4;

      if (rateCount < 4) {
        rateCount++;
      }

      int sum = 0;

      for (byte i = 0; i < rateCount; i++) {
        sum += rates[i];
      }

      currentHR = (sum + rateCount / 2) / rateCount;
      hrValid = true;
    }
  }

  if (millis() - lastIR > 1000 ||
      (haveBeat && millis() - lastBeat > 3000)) {
    resetPulse();
  }
}

// --------------------------------------------------
// RESTING HEART RATE CALIBRATION
// --------------------------------------------------

int sampleCount = 0;
int sampleSum = 0;

unsigned long lastCalSample = 0;

void calibrateStep() {
  if (!hrValid) {
    sampleCount = 0;
    sampleSum = 0;
    lastCalSample = millis();
    return;
  }

  if (millis() - lastCalSample >= 1000UL) {
    lastCalSample = millis();

    sampleCount++;
    sampleSum += currentHR;

    Serial.print(F("CAL SAMPLE "));
    Serial.print(sampleCount);
    Serial.print(F("/5 HR="));
    Serial.println(currentHR);

    if (sampleCount >= 5) {
      restingHR = (sampleSum + 2) / 5;
      restingHRReady = true;

      Serial.print(F("CAL COMPLETE REST="));
      Serial.print(restingHR);

      Serial.print(F(" HR_TRIGGER_ABOVE="));
      Serial.println(
        restingHR * HR_TRIGGER_MULTIPLIER,
        1
      );

      enter(RESTING);
    }
  }
}

// --------------------------------------------------
// HEART RATE TRIGGER
// --------------------------------------------------

bool hrElevated() {
  return restingHRReady &&
         hrValid &&
         (float)currentHR >
         HR_TRIGGER_MULTIPLIER * (float)restingHR;
}

// --------------------------------------------------
// CONTINUOUS MOVEMENT TRIGGER (STAGE 1)
// --------------------------------------------------

bool sustained(bool condition, unsigned long duration) {
  if (!condition) {
    elevatedTracking = false;
    return false;
  }

  if (!elevatedTracking) {
    elevatedTracking = true;
    elevatedSince = millis();
  }

  return millis() - elevatedSince >= duration;
}

// --------------------------------------------------
// STAGE 1 VIBRATION
// --------------------------------------------------

void stage1Motor() {
  // Gentle vibration pulses for 5 seconds
  motor(
    ((millis() - modeStart) % 600UL) < 180UL
      ? 105
      : 0
  );
}

// --------------------------------------------------
// STAGE 2 VIBRATION
// --------------------------------------------------

void stage2Motor() {
  // Stronger vibration until reset button
  motor(
    ((millis() - modeStart) % 800UL) < 500UL
      ? 190
      : 0
  );
}

// --------------------------------------------------
// RETURN TO NORMAL MODE
// --------------------------------------------------

void ordinaryMode() {
  if (!restingHRReady) {
    enter(CALIBRATING);
    return;
  }

  if (!hrValid) {
    enter(NO_SIGNAL);
    return;
  }

  if (currentHR <= restingHR - 10) {
    enter(SLEEPING);
  } else {
    enter(RESTING);
  }
}

// --------------------------------------------------
// STATE MACHINE
// --------------------------------------------------

void runDemo() {
  unsigned long age = millis() - modeStart;

  switch (mode) {

    // CALIBRATION
    case CALIBRATING:
      calibrateStep();
      break;

    // RESTING OR SLEEPING
    case RESTING:
    case SLEEPING:

      if (!hrValid) {
        enter(NO_SIGNAL);
        break;
      }

      // Stage 1 triggers if:
      // HR > 1.4 times resting
      // OR movement > 0.45 for 3 seconds

      if (
        hrElevated() ||
        sustained(
          mpuOK && motion > MOVEMENT_THRESHOLD,
          TRIGGER_MS
        )
      ) {
        enter(STAGE1);
      }

      else if (!elevatedTracking) {
        Mode target =
          currentHR <= restingHR - 10
          ? SLEEPING
          : RESTING;

        if (mode != target) {
          enter(target);
        }
      }

      break;

    // NO HR SIGNAL
    case NO_SIGNAL:

      if (hrValid) {
        if (restingHRReady) {
          ordinaryMode();
        } else {
          sampleCount = 0;
          sampleSum = 0;
          lastCalSample = millis();

          enter(CALIBRATING);
        }
      }

      break;

    // STAGE 1
    case STAGE1:

      if (age >= STAGE1_MS) {
        enter(COOLDOWN);
      } else {
        stage1Motor();
      }

      break;

    // COOLDOWN
    case COOLDOWN:

      if (age >= COOLDOWN_MS) {
        if (restingHRReady) {
          enter(MONITORING);
        } else {
          enter(CALIBRATING);
        }
      }

      break;

    // MONITORING AFTER COOLDOWN
    case MONITORING: {

      // The rolling window is updated every 50 ms
      // in loop() during monitoring.

      // Wait for the full 15-second observing period.
      if (age < OBSERVE_MS) {
        break;
      }

      // STAGE 2 TRIGGER:
      // Heart rate > 1.4x resting
      // OR at least 54/60 movement readings > 0.45

      if (
        hrElevated() ||
        movementWindowTriggered()
      ) {
        Serial.println(
          F("TRIGGER STAGE2: HR or 3s rolling motion")
        );

        enter(STAGE2);
        break;
      }

      // Return to normal only after movement
      // has been low for 3 seconds and HR is normal.

      if (
        mpuOK &&
        motion <= MOVEMENT_THRESHOLD &&
        !hrElevated()
      ) {

        if (lowSince == 0) {
          lowSince = millis();
        }

        if (millis() - lowSince >= TRIGGER_MS) {
          ordinaryMode();
        }

      } else {
        lowSince = 0;
      }

      break;
    }

    // STAGE 2
    case STAGE2:
      stage2Motor();
      break;
  }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  motor(0);

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000);

  lcd.begin(16, 2);

  Serial.println(F("DreamGuard"));

  startMax();

  mpuOK =
    mpuWrite(0x6B, 0x01) &&
    mpuWrite(0x1C, 0);

  Serial.print(F("MPU_INIT="));
  Serial.println(mpuOK ? 1 : 0);

  enter(CALIBRATING);
  displayStatus();
}

// --------------------------------------------------
// MAIN LOOP
// --------------------------------------------------

void loop() {
  unsigned long now = millis();

  // RESET BUTTON
  if (buttonPressed()) {
    enter(COOLDOWN);
  }

  // HEART RATE
  readMax();

  // MOVEMENT: every 50 ms
  if (now - lastMotion >= 50) {
    lastMotion = now;

    realMotion();

    // Rolling window only after cooldown
    if (mode == MONITORING) {
      recordMovementSample();
    }
  }

  // STATE MACHINE
  runDemo();

  // LCD: every 250 ms
  if (now - lastDisplay >= 250) {
    lastDisplay = now;
    displayStatus();
  }

  // SERIAL: every 1000 ms
  if (now - lastLog >= 1000) {
    lastLog = now;
    outputLog();
  }
}
