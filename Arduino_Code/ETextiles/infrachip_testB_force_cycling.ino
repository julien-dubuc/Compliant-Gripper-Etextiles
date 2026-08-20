#include <Servo.h>
#include "HX711.h"

// ============================================================
// INFRACHIP TEST B - FORCE CYCLING / HYSTERESIS TEST
// Caliper-based length measurement version
// Arduino UNO R4 WiFi
//
// Serial output for MATLAB:
// DATA,time_ms,force_N,Rx_ohm,state,cycle,target_force_N,measure_needed,measure_ready,hold_elapsed_s
//
// Buttons:
// D7 -> start/abort protocol, button to GND
// D4 -> tare load cell, button to GND, only when protocol is not running
//
// Optional cue:
// D8 -> HIGH when Julien should take the caliper measurement
//
// Protocol:
// 1) Mount fresh 50 mm specimen.
// 2) Start MATLAB recording.
// 3) Press D7.
// 4) Stretch closed-loop until 1.5 N.
// 5) Hold at 1.5 N for dwell.
// 6) Stretch closed-loop until 4.5 N.
// 7) Hold at 4.5 N for dwell.
// 8) Release closed-loop until 1.5 N.
// 9) Hold at 1.5 N for dwell.
// 10) Repeat 4.5 N / 1.5 N cycle 20 times.
// 11) Release to ~0 N and add 3 reverse rotations for slack.
// ============================================================


// ============================================================
// USER SETTINGS
// ============================================================

const unsigned long sensorIntervalMs = 100;  // 10 Hz

const int adcBits = 14;
const int adcMaxValue = 16383;


// ============================================================
// TEST B SETTINGS
// ============================================================

const float lowForceN = 1.5;
const float highForceN = 4.5;

// Dwell at 1.5 N and 4.5 N.
// Increased to 45 s to allow manual caliper measurement.
const unsigned long dwellMs = 30000;

const int totalCycles = 20;

// During each selected measurement hold, wait this long before asking
// Julien to measure with the caliper. This allows the force/resistance
// to settle after the movement.
const unsigned long caliperCueDelayMs = 30000;

// Closed-loop force tolerance.
const float forceToleranceN = 0.08;

// Hard force protection.
const float forceLimitN = 8.0;

// End release settings
const bool releaseAtEnd = true;
const float zeroForceThresholdN = 0.03;
const float extraSlackRotations = 3.0;

// Cycles where manual length should be measured.
// cycle 0 = initial 1.5 N hold before the 20 cycles start.
// For cycles >0, measure both 4.5 N HIGH_HOLD and 1.5 N LOW_HOLD.
const int caliperMeasureCycles[] = {0, 1, 5, 10, 15, 20};
const int numberOfCaliperMeasureCycles =
  sizeof(caliperMeasureCycles) / sizeof(caliperMeasureCycles[0]);


// ============================================================
// SERVO SETTINGS
// ============================================================

// If stretching moves the wrong way, change this to -1.
const int stretchDirection = 1;

// Continuous servo stop pulse.
int servoStop = 1520;

const int servoMin = 1000;
const int servoMax = 2000;

// Approximate speed control.
// Larger offset = faster. Smaller offset = slower.
const int autoMoveOffsetUs = 25;

// Smaller correction during holds to reduce oscillation while measuring.
const int holdCorrectionOffsetUs = 18;

// Calibrate this for the end extra-slack reverse movement.
const float secondsPerRotationRelax = 4.0;


// ============================================================
// PINS
// ============================================================

const int servoPin = 9;
const int joystickPin = A1;

const int hx711DoutPin = 2;
const int hx711SckPin  = 3;

const int tareButtonPin = 4;
const int protocolButtonPin = 7;

// Optional LED/buzzer/DAQ cue for manual caliper measurement.
const int measurementCuePin = 8;

const int ad623OutPin = A0;
const int ad623RefPin = A2;
const int bridgeSensePin = A3;


// ============================================================
// JOYSTICK SETTINGS
// ============================================================

Servo myServo;

int joystickCenter = 8192;
const int joystickDeadband = 400;


// ============================================================
// HX711 LOAD CELL SETTINGS
// ============================================================

HX711 scale;

const float newtonPerRawValue = 4.8598932385e-6;

// If force is negative when applying load, change this to -1.0.
const float loadDirection = 1.0;

const int loadCellSamples = 1;


// ============================================================
// WHEATSTONE BRIDGE + AD623 SETTINGS
// ============================================================

const float R1 = 1000.0;
const float R2 = 1000.0;
const float R3 = 250.0;
const float R4 = 250.0;

// Rg = 10.9 kOhm
// Gain = 1 + 100000 / 10900 = 10.174
const float ad623Gain = 10.174;

const float bridgePolarity = 1.0;

// Bridge +5 V ---- 10k ---- A3 ---- 10k ---- GND
const float bridgeSenseMultiplier = 2.0;

const int analogSamples = 20;


// ============================================================
// RESISTANCE CALIBRATION
// ============================================================

const bool useResistanceCalibration = true;

const float rawRxAtShort = -58.0;
const float rawRxAtKnown = -34.0;
const float knownRxOhm = 38.8;


// ============================================================
// PROTOCOL STATE
// ============================================================

enum ProtocolState {
  STATE_IDLE,
  STATE_GO_TO_INITIAL_LOW,
  STATE_INITIAL_LOW_HOLD,
  STATE_GO_TO_HIGH,
  STATE_HIGH_HOLD,
  STATE_GO_TO_LOW,
  STATE_LOW_HOLD,
  STATE_FORCE_LIMIT_ABORT,
  STATE_RETURN_TO_ZERO,
  STATE_EXTRA_SLACK,
  STATE_DONE,
  STATE_ABORTED
};

ProtocolState protocolState = STATE_IDLE;

unsigned long previousSensorReadTime = 0;
unsigned long stateStartMs = 0;

int currentCycle = 0;
float currentTargetForceN = 0.0;

// Timed end slack movement
bool timedMoveActive = false;
unsigned long timedMoveStartMs = 0;
unsigned long timedMoveDurationMs = 0;
int timedMoveCommandUs = 1520;

// Latest measurements
float latestForceN = 0.0;
float latestRxOhm = 0.0;
bool latestForceValid = false;
bool latestResistanceValid = false;


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(adcBits);

  pinMode(tareButtonPin, INPUT_PULLUP);
  pinMode(protocolButtonPin, INPUT_PULLUP);
  pinMode(measurementCuePin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(measurementCuePin, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  myServo.attach(servoPin);
  stopServo();

  calibrateJoystick();

  scale.begin(hx711DoutPin, hx711SckPin);

  delay(1000);
  scale.tare();

  Serial.println("DATA,time_ms,force_N,Rx_ohm,state,cycle,target_force_N,measure_needed,measure_ready,hold_elapsed_s");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  handleTareButton();
  handleProtocolButton();

  readSensorsAtInterval();
  updateProtocol();
  updateMeasurementCue();

  // Manual joystick control when protocol is not running.
  if (!isProtocolRunning()) {
    controlServoWithJoystick();
  }
}


// ============================================================
// JOYSTICK + MANUAL SERVO CONTROL
// ============================================================

void calibrateJoystick() {
  long sum = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(joystickPin);
    delay(5);
  }

  joystickCenter = sum / samples;
}

void controlServoWithJoystick() {
  int joystickValue = analogRead(joystickPin);
  int difference = joystickValue - joystickCenter;

  int pulseWidth = servoStop;

  if (abs(difference) < joystickDeadband) {
    pulseWidth = servoStop;
  } else {
    if (difference > 0) {
      pulseWidth = map(
        difference,
        joystickDeadband,
        adcMaxValue - joystickCenter,
        servoStop + 20,
        servoMax
      );
    } else {
      pulseWidth = map(
        difference,
        -joystickDeadband,
        -joystickCenter,
        servoStop - 20,
        servoMin
      );
    }
  }

  pulseWidth = constrain(pulseWidth, servoMin, servoMax);
  myServo.writeMicroseconds(pulseWidth);
}

void stopServo() {
  myServo.writeMicroseconds(servoStop);
  timedMoveActive = false;
}


// ============================================================
// BUTTONS
// ============================================================

void handleTareButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastChangeTime = 0;

  const unsigned long debounceMs = 50;

  bool reading = digitalRead(tareButtonPin);

  if (reading != lastReading) {
    lastChangeTime = millis();
    lastReading = reading;
  }

  if ((millis() - lastChangeTime) > debounceMs) {
    if (reading != stableState) {
      stableState = reading;

      if (stableState == LOW) {
        if (!isProtocolRunning()) {
          scale.tare();
        }
      }
    }
  }
}

void handleProtocolButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastChangeTime = 0;

  const unsigned long debounceMs = 50;

  bool reading = digitalRead(protocolButtonPin);

  if (reading != lastReading) {
    lastChangeTime = millis();
    lastReading = reading;
  }

  if ((millis() - lastChangeTime) > debounceMs) {
    if (reading != stableState) {
      stableState = reading;

      if (stableState == LOW) {
        if (isProtocolRunning()) {
          abortProtocol();
        } else {
          startProtocol();
        }
      }
    }
  }
}

bool isProtocolRunning() {
  return (
    protocolState == STATE_GO_TO_INITIAL_LOW ||
    protocolState == STATE_INITIAL_LOW_HOLD ||
    protocolState == STATE_GO_TO_HIGH ||
    protocolState == STATE_HIGH_HOLD ||
    protocolState == STATE_GO_TO_LOW ||
    protocolState == STATE_LOW_HOLD ||
    protocolState == STATE_FORCE_LIMIT_ABORT ||
    protocolState == STATE_RETURN_TO_ZERO ||
    protocolState == STATE_EXTRA_SLACK
  );
}


// ============================================================
// PROTOCOL CONTROL
// ============================================================

void startProtocol() {
  stopServo();

  currentCycle = 0;
  currentTargetForceN = lowForceN;

  protocolState = STATE_GO_TO_INITIAL_LOW;
  stateStartMs = millis();

  digitalWrite(LED_BUILTIN, HIGH);
}

void abortProtocol() {
  stopServo();

  protocolState = STATE_ABORTED;
  stateStartMs = millis();

  currentTargetForceN = 0.0;

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(measurementCuePin, LOW);
}

void finishProtocol() {
  stopServo();

  protocolState = STATE_DONE;
  stateStartMs = millis();

  currentTargetForceN = 0.0;

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(measurementCuePin, LOW);
}

void forceLimitAbort() {
  stopServo();

  protocolState = STATE_FORCE_LIMIT_ABORT;
  stateStartMs = millis();

  currentTargetForceN = 0.0;

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(measurementCuePin, LOW);
}

void updateProtocol() {
  unsigned long nowMs = millis();

  if (!isProtocolRunning()) {
    return;
  }

  if (!latestForceValid) {
    stopServo();
    return;
  }

  // Hard force protection.
  if (latestForceN >= forceLimitN &&
      protocolState != STATE_RETURN_TO_ZERO &&
      protocolState != STATE_EXTRA_SLACK) {
    forceLimitAbort();
    return;
  }

  switch (protocolState) {

    case STATE_GO_TO_INITIAL_LOW:
      currentTargetForceN = lowForceN;

      if (latestForceN >= lowForceN - forceToleranceN) {
        stopServo();
        protocolState = STATE_INITIAL_LOW_HOLD;
        stateStartMs = nowMs;
      } else {
        moveTowardForceTarget(lowForceN);
      }
      break;

    case STATE_INITIAL_LOW_HOLD:
      currentTargetForceN = lowForceN;
      holdForceAroundTarget(lowForceN);

      if (nowMs - stateStartMs >= dwellMs) {
        currentCycle = 1;
        protocolState = STATE_GO_TO_HIGH;
        stateStartMs = nowMs;
      }
      break;

    case STATE_GO_TO_HIGH:
      currentTargetForceN = highForceN;

      if (latestForceN >= highForceN - forceToleranceN) {
        stopServo();
        protocolState = STATE_HIGH_HOLD;
        stateStartMs = nowMs;
      } else {
        moveTowardForceTarget(highForceN);
      }
      break;

    case STATE_HIGH_HOLD:
      currentTargetForceN = highForceN;
      holdForceAroundTarget(highForceN);

      if (nowMs - stateStartMs >= dwellMs) {
        protocolState = STATE_GO_TO_LOW;
        stateStartMs = nowMs;
      }
      break;

    case STATE_GO_TO_LOW:
      currentTargetForceN = lowForceN;

      if (latestForceN <= lowForceN + forceToleranceN) {
        stopServo();
        protocolState = STATE_LOW_HOLD;
        stateStartMs = nowMs;
      } else {
        moveTowardForceTarget(lowForceN);
      }
      break;

    case STATE_LOW_HOLD:
      currentTargetForceN = lowForceN;
      holdForceAroundTarget(lowForceN);

      if (nowMs - stateStartMs >= dwellMs) {
        if (currentCycle >= totalCycles) {
          if (releaseAtEnd) {
            protocolState = STATE_RETURN_TO_ZERO;
            stateStartMs = nowMs;
            currentTargetForceN = 0.0;
          } else {
            finishProtocol();
          }
        } else {
          currentCycle++;
          protocolState = STATE_GO_TO_HIGH;
          stateStartMs = nowMs;
        }
      }
      break;

    case STATE_FORCE_LIMIT_ABORT:
      stopServo();

      // Safety abort: release after 2 s.
      if (nowMs - stateStartMs >= 2000) {
        protocolState = STATE_RETURN_TO_ZERO;
        stateStartMs = nowMs;
      }
      break;

    case STATE_RETURN_TO_ZERO:
      updateReturnToZero();
      break;

    case STATE_EXTRA_SLACK:
      updateTimedMove();

      if (!timedMoveActive) {
        finishProtocol();
      }
      break;

    default:
      stopServo();
      break;
  }
}


// ============================================================
// FORCE CONTROL
// ============================================================

void moveTowardForceTarget(float targetN) {
  if (latestForceN < targetN - forceToleranceN) {
    stretchServo(autoMoveOffsetUs);
  } else if (latestForceN > targetN + forceToleranceN) {
    relaxServo(autoMoveOffsetUs);
  } else {
    stopServo();
  }
}

void holdForceAroundTarget(float targetN) {
  if (latestForceN < targetN - forceToleranceN) {
    stretchServo(holdCorrectionOffsetUs);
  } else if (latestForceN > targetN + forceToleranceN) {
    relaxServo(holdCorrectionOffsetUs);
  } else {
    stopServo();
  }
}

void stretchServo(int offsetUs) {
  int command = servoStop + stretchDirection * offsetUs;
  command = constrain(command, servoMin, servoMax);
  myServo.writeMicroseconds(command);
}

void relaxServo(int offsetUs) {
  int command = servoStop - stretchDirection * offsetUs;
  command = constrain(command, servoMin, servoMax);
  myServo.writeMicroseconds(command);
}

void updateReturnToZero() {
  if (latestForceN <= zeroForceThresholdN) {
    stopServo();

    startTimedRelaxMove(extraSlackRotations);

    protocolState = STATE_EXTRA_SLACK;
    stateStartMs = millis();
    return;
  }

  relaxServo(autoMoveOffsetUs);
}


// ============================================================
// TIMED EXTRA SLACK MOTION
// ============================================================

void startTimedRelaxMove(float rotations) {
  if (rotations <= 0.0) {
    timedMoveActive = false;
    stopServo();
    return;
  }

  float durationSec = rotations * secondsPerRotationRelax;
  timedMoveDurationMs = (unsigned long)(durationSec * 1000.0);

  if (timedMoveDurationMs < 1) {
    timedMoveDurationMs = 1;
  }

  timedMoveCommandUs = servoStop - stretchDirection * autoMoveOffsetUs;
  timedMoveCommandUs = constrain(timedMoveCommandUs, servoMin, servoMax);

  timedMoveStartMs = millis();
  timedMoveActive = true;

  myServo.writeMicroseconds(timedMoveCommandUs);
}

void updateTimedMove() {
  if (!timedMoveActive) {
    return;
  }

  unsigned long nowMs = millis();

  if (nowMs - timedMoveStartMs >= timedMoveDurationMs) {
    stopServo();
    timedMoveActive = false;
  } else {
    myServo.writeMicroseconds(timedMoveCommandUs);
  }
}


// ============================================================
// MANUAL LENGTH MEASUREMENT CUE
// ============================================================

bool isHoldState() {
  return (
    protocolState == STATE_INITIAL_LOW_HOLD ||
    protocolState == STATE_HIGH_HOLD ||
    protocolState == STATE_LOW_HOLD
  );
}

bool isSelectedCaliperCycle(int cycleNumber) {
  for (int i = 0; i < numberOfCaliperMeasureCycles; i++) {
    if (caliperMeasureCycles[i] == cycleNumber) {
      return true;
    }
  }
  return false;
}

bool measureNeededNow() {
  if (!isHoldState()) {
    return false;
  }

  if (protocolState == STATE_INITIAL_LOW_HOLD) {
    return isSelectedCaliperCycle(0);
  }

  if (protocolState == STATE_HIGH_HOLD || protocolState == STATE_LOW_HOLD) {
    return isSelectedCaliperCycle(currentCycle);
  }

  return false;
}

float currentHoldElapsedSec() {
  if (!isHoldState()) {
    return 0.0;
  }

  return (millis() - stateStartMs) / 1000.0;
}

bool measureReadyNow() {
  if (!measureNeededNow()) {
    return false;
  }

  return (millis() - stateStartMs) >= caliperCueDelayMs;
}

void updateMeasurementCue() {
  if (measureReadyNow()) {
    digitalWrite(measurementCuePin, HIGH);
  } else {
    digitalWrite(measurementCuePin, LOW);
  }
}


// ============================================================
// SENSOR READING
// ============================================================

void readSensorsAtInterval() {
  unsigned long currentTime = millis();

  if (currentTime - previousSensorReadTime >= sensorIntervalMs) {
    previousSensorReadTime = currentTime;

    latestForceValid = false;
    latestResistanceValid = false;

    if (scale.is_ready()) {
      float rawLoadCell = scale.get_value(loadCellSamples);
      latestForceN = rawLoadCell * newtonPerRawValue * loadDirection;
      latestForceValid = true;
    }

    float rxRawOhm = 0.0;
    float rxCalibratedOhm = 0.0;

    latestResistanceValid =
      calculateRxResistance(rxRawOhm, rxCalibratedOhm);

    if (latestResistanceValid) {
      latestRxOhm = rxCalibratedOhm;
    }

    if (latestForceValid && latestResistanceValid) {
      Serial.print("DATA,");
      Serial.print(currentTime);
      Serial.print(",");
      Serial.print(latestForceN, 3);
      Serial.print(",");
      Serial.print(latestRxOhm, 3);
      Serial.print(",");
      Serial.print(stateName(protocolState));
      Serial.print(",");
      Serial.print(currentCycle);
      Serial.print(",");
      Serial.print(currentTargetForceN, 3);
      Serial.print(",");
      Serial.print(measureNeededNow() ? 1 : 0);
      Serial.print(",");
      Serial.print(measureReadyNow() ? 1 : 0);
      Serial.print(",");
      Serial.println(currentHoldElapsedSec(), 2);
    }
  }
}


// ============================================================
// ANALOG READING
// ============================================================

float readAnalogAverageCounts(int pin) {
  long sum = 0;

  analogRead(pin);
  delayMicroseconds(500);

  for (int i = 0; i < analogSamples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }

  return sum / float(analogSamples);
}


// ============================================================
// AD623 + BRIDGE CALCULATION
// ============================================================

bool calculateRxResistance(float &rxRawOhm, float &rxCalibratedOhm) {
  float ad623OutCounts = readAnalogAverageCounts(ad623OutPin);
  float ad623RefCounts = readAnalogAverageCounts(ad623RefPin);
  float bridgeSenseCounts = readAnalogAverageCounts(bridgeSensePin);

  float bridgeSupplyCounts = bridgeSenseCounts * bridgeSenseMultiplier;

  if (bridgeSupplyCounts <= 0.0) {
    return false;
  }

  float vdiffCounts =
    bridgePolarity *
    (ad623OutCounts - ad623RefCounts) /
    ad623Gain;

  float nodeARatio = R3 / (R1 + R3);
  float nodeBRatio = nodeARatio + (vdiffCounts / bridgeSupplyCounts);

  if (nodeBRatio <= 0.0 || nodeBRatio >= 1.0) {
    return false;
  }

  float rightBottomResistance =
    R2 * nodeBRatio / (1.0 - nodeBRatio);

  rxRawOhm = rightBottomResistance - R4;

  if (useResistanceCalibration) {
    rxCalibratedOhm =
      (rxRawOhm - rawRxAtShort) *
      knownRxOhm /
      (rawRxAtKnown - rawRxAtShort);
  } else {
    rxCalibratedOhm = rxRawOhm;
  }

  return true;
}


// ============================================================
// STATE NAME FOR CSV
// ============================================================

const char* stateName(ProtocolState s) {
  switch (s) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_GO_TO_INITIAL_LOW:
      return "GO_TO_INITIAL_LOW";
    case STATE_INITIAL_LOW_HOLD:
      return "INITIAL_LOW_HOLD";
    case STATE_GO_TO_HIGH:
      return "GO_TO_HIGH";
    case STATE_HIGH_HOLD:
      return "HIGH_HOLD";
    case STATE_GO_TO_LOW:
      return "GO_TO_LOW";
    case STATE_LOW_HOLD:
      return "LOW_HOLD";
    case STATE_FORCE_LIMIT_ABORT:
      return "FORCE_LIMIT_ABORT";
    case STATE_RETURN_TO_ZERO:
      return "RETURN_TO_ZERO";
    case STATE_EXTRA_SLACK:
      return "EXTRA_SLACK";
    case STATE_DONE:
      return "DONE";
    case STATE_ABORTED:
      return "ABORTED";
    default:
      return "UNKNOWN";
  }
}
