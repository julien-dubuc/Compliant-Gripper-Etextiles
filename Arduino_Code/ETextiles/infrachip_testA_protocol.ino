#include <Servo.h>
#include "HX711.h"

// ============================================================
// TEST A-LIKE STRAIN PROTOCOL WITH PRELOAD MEASUREMENT
// Arduino UNO R4 WiFi
//
// Output format for MATLAB:
// DATA,time_ms,force_N,Rx_ohm,state,step,target_strain_pct
//
// Start/abort full procedure: D7 button to GND
// Tare load cell: D4 button to GND, only when not running
//
// Protocol:
// 1) Preload until force reaches 0.1 N
// 2) Hold preload for 15 s and measure this as an additional point
// 3) Move through strain steps: 0.2, 0.4, 0.6, 0.8, 1, 2, 3, 4, 5%
// 4) Hold intermediate strain steps for 15 s
// 5) Hold final 5% step for 5 min
// 6) Return until force is ~0 N
// 7) Continue relaxing for 3 extra rotations to create slack
//
// IMPORTANT:
// This routine uses timed servo rotations. A continuous rotation servo
// has no position feedback, so calibrate secondsPerRotationStretch/Relax
// for your exact mechanism and servo pulse command.
// ============================================================


// ============================================================
// USER SETTINGS
// ============================================================

const unsigned long sensorIntervalMs = 100;  // 10 Hz

const int adcBits = 14;
const int adcMaxValue = 16383;


// ============================================================
// MECHANICAL / TEST PROTOCOL SETTINGS
// ============================================================

const float originalLengthMm = 70.0;  // 7 cm = central 5 cm + 2 cm hooks
const float mmPerServoRotation = 0.48;

// Preload to remove backlash/play
const float preloadForceN = 0.10;

// New requested preload hold.
// This is analysed as an extra measurement point.
const unsigned long preloadHoldMs = 15000;  // 15 seconds

// Step targets, cumulative engineering strain in %
const float strainTargetsPct[] = {
  0.2, 0.4, 0.6, 0.8, 1.0, 2.0, 3.0, 4.0, 5.0
};

const int numberOfStrainSteps =
  sizeof(strainTargetsPct) / sizeof(strainTargetsPct[0]);

// Hold at each intermediate step
const unsigned long stepHoldMs = 15000;

// Final 5% hold
const unsigned long finalHoldMs = 5UL * 60UL * 1000UL;  // 5 minutes

// If true, 5% gets 15 s hold first, then 5 min final hold.
// If false, 5% directly starts the 5 min hold.
const bool include15sHoldAtFinalStep = false;

// Return phase
const float zeroForceThresholdN = 0.03;  // practical 0 N threshold
const float extraSlackRotations = 3.0;   // after reaching ~0 N

// Hard force limit behaviour.
// If force reaches this threshold during preload/move/hold:
// stop, hold for 5 min, then return to zero and add slack.
const float forceLimitN = 8.0;


// ============================================================
// SERVO MOTION CALIBRATION
// ============================================================

// If stretching moves the wrong way, change this to -1.
const int stretchDirection = 1;

// Continuous servo stop pulse, already calibrated.
int servoStop = 1520;

// Servo limits
const int servoMin = 1000;
const int servoMax = 2000;

// Approximate speed control.
// Larger offset = faster.
// Smaller offset = slower, but too small may stall.
const int autoMoveOffsetUs = 15;
const int preloadOffsetUs = 20;

// Time for one full output rotation at servoStop +/- autoMoveOffsetUs.
// YOU MUST CALIBRATE THESE VALUES ON YOUR MECHANISM.
const float secondsPerRotationStretch = 4.0;
const float secondsPerRotationRelax   = 4.0;


// ============================================================
// PINS
// ============================================================

// Servo + joystick
const int servoPin = 9;
const int joystickPin = A1;

// HX711
const int hx711DoutPin = 2;
const int hx711SckPin  = 3;

// Buttons
const int tareButtonPin = 4;
const int protocolButtonPin = 7;

// AD623 / Wheatstone bridge
const int ad623OutPin = A0;
const int ad623RefPin = A2;
const int bridgeSensePin = A3;  // Bridge +5 V through 10k/10k divider


// ============================================================
// SERVO + JOYSTICK SETTINGS
// ============================================================

Servo myServo;

int joystickCenter = 8192;
const int joystickDeadband = 400;  // reduced for easier manual motion


// ============================================================
// HX711 LOAD CELL SETTINGS
// ============================================================

HX711 scale;

// Your HX711/load-cell sensitivity:
// 4.8598932385e-6 N per raw HX711 value
const float newtonPerRawValue = 4.8598932385e-6;

// If force is negative when applying load, change this to -1.0
const float loadDirection = 1.0;

// Use 1 for faster reading.
// Increase to 2-5 if force is too noisy.
const int loadCellSamples = 1;


// ============================================================
// WHEATSTONE BRIDGE + AD623 SETTINGS
// ============================================================

// Bridge resistors
const float R1 = 1000.0;  // Ohm
const float R2 = 1000.0;  // Ohm
const float R3 = 250.0;   // Ohm
const float R4 = 250.0;   // Ohm

// Measured AD623 gain resistor:
// Rg = 10.9 kOhm
// Gain = 1 + 100000 / 10900 = 10.174
const float ad623Gain = 10.174;

// If Rx changes in the wrong direction, change this to -1.0
const float bridgePolarity = 1.0;

// A3 bridge supply sense:
// Bridge +5 V ---- 10k ---- A3 ---- 10k ---- GND
const float bridgeSenseMultiplier = 2.0;

// Use 20 for fast/acceptable smoothing.
// Increase to 50 if resistance is too noisy.
const int analogSamples = 20;


// ============================================================
// TWO-POINT RESISTANCE CALIBRATION
// ============================================================

// Previous measured raw values:
// Short circuit gave -58 Ohm
// 38.8 Ohm fixed resistor gave -34 Ohm
//
// This maps:
// raw -58  -> 0 Ohm
// raw -34  -> 38.8 Ohm

const bool useResistanceCalibration = true;

const float rawRxAtShort = -58.0;
const float rawRxAtKnown = -34.0;
const float knownRxOhm = 38.8;


// ============================================================
// PROTOCOL STATE
// ============================================================

enum ProtocolState {
  STATE_IDLE,
  STATE_PRELOAD,
  STATE_PRELOAD_HOLD,
  STATE_STEP_MOVE,
  STATE_STEP_HOLD,
  STATE_FINAL_HOLD,
  STATE_FORCE_LIMIT_HOLD,
  STATE_RETURN_TO_ZERO,
  STATE_EXTRA_SLACK,
  STATE_DONE,
  STATE_ABORTED
};

ProtocolState protocolState = STATE_IDLE;

unsigned long previousSensorReadTime = 0;
unsigned long stateStartMs = 0;

// Current step
int currentStepIndex = -1;
float currentStrainPct = 0.0;
float targetStrainPct = 0.0;

// Timed motion
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
  pinMode(LED_BUILTIN, OUTPUT);

  myServo.attach(servoPin);
  stopServo();

  calibrateJoystick();

  scale.begin(hx711DoutPin, hx711SckPin);

  delay(1000);
  scale.tare();

  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("DATA,time_ms,force_N,Rx_ohm,state,step,target_strain_pct");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  handleTareButton();
  handleProtocolButton();

  readSensorsAtInterval();
  updateProtocol();

  // Joystick remains active whenever the routine is not running.
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
    protocolState == STATE_PRELOAD ||
    protocolState == STATE_PRELOAD_HOLD ||
    protocolState == STATE_STEP_MOVE ||
    protocolState == STATE_STEP_HOLD ||
    protocolState == STATE_FINAL_HOLD ||
    protocolState == STATE_FORCE_LIMIT_HOLD ||
    protocolState == STATE_RETURN_TO_ZERO ||
    protocolState == STATE_EXTRA_SLACK
  );
}


// ============================================================
// PROTOCOL CONTROL
// ============================================================

void startProtocol() {
  stopServo();

  currentStepIndex = -1;
  currentStrainPct = 0.0;
  targetStrainPct = 0.0;

  protocolState = STATE_PRELOAD;
  stateStartMs = millis();

  digitalWrite(LED_BUILTIN, HIGH);
}

void abortProtocol() {
  stopServo();
  protocolState = STATE_ABORTED;
  stateStartMs = millis();
  digitalWrite(LED_BUILTIN, LOW);
}

void finishProtocol() {
  stopServo();
  protocolState = STATE_DONE;
  stateStartMs = millis();
  digitalWrite(LED_BUILTIN, LOW);
}

void enterForceLimitHold() {
  stopServo();
  protocolState = STATE_FORCE_LIMIT_HOLD;
  stateStartMs = millis();
  digitalWrite(LED_BUILTIN, HIGH);
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

  // Hard stop behaviour:
  // If force exceeds 8 N during stretching/holding, stop immediately,
  // hold for 5 min, then return to zero and add slack.
  if (
    protocolState == STATE_PRELOAD ||
    protocolState == STATE_PRELOAD_HOLD ||
    protocolState == STATE_STEP_MOVE ||
    protocolState == STATE_STEP_HOLD ||
    protocolState == STATE_FINAL_HOLD
  ) {
    if (latestForceN >= forceLimitN) {
      enterForceLimitHold();
      return;
    }
  }

  switch (protocolState) {

    case STATE_PRELOAD:
      updatePreload();
      break;

    case STATE_PRELOAD_HOLD:
      stopServo();
      if (nowMs - stateStartMs >= preloadHoldMs) {
        currentStepIndex = 0;
        startStepMove(currentStepIndex);
      }
      break;

    case STATE_STEP_MOVE:
      updateTimedMove();

      if (!timedMoveActive) {
        currentStrainPct = targetStrainPct;

        bool isFinalStep = (currentStepIndex == numberOfStrainSteps - 1);

        if (isFinalStep && !include15sHoldAtFinalStep) {
          protocolState = STATE_FINAL_HOLD;
          stateStartMs = nowMs;
        } else {
          protocolState = STATE_STEP_HOLD;
          stateStartMs = nowMs;
        }
      }
      break;

    case STATE_STEP_HOLD:
      stopServo();

      if (nowMs - stateStartMs >= stepHoldMs) {
        bool isFinalStep = (currentStepIndex == numberOfStrainSteps - 1);

        if (isFinalStep) {
          protocolState = STATE_FINAL_HOLD;
          stateStartMs = nowMs;
        } else {
          currentStepIndex++;
          startStepMove(currentStepIndex);
        }
      }
      break;

    case STATE_FINAL_HOLD:
      stopServo();

      if (nowMs - stateStartMs >= finalHoldMs) {
        protocolState = STATE_RETURN_TO_ZERO;
        stateStartMs = nowMs;
      }
      break;

    case STATE_FORCE_LIMIT_HOLD:
      stopServo();

      if (nowMs - stateStartMs >= finalHoldMs) {
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

void updatePreload() {
  if (latestForceN >= preloadForceN) {
    stopServo();

    // New behaviour: hold at 0.1 N for 15 s and log it as PRELOAD_HOLD.
    protocolState = STATE_PRELOAD_HOLD;
    stateStartMs = millis();
    currentStepIndex = -1;
    targetStrainPct = 0.0;
    return;
  }

  int command = servoStop + stretchDirection * preloadOffsetUs;
  command = constrain(command, servoMin, servoMax);
  myServo.writeMicroseconds(command);
}

void startStepMove(int stepIndex) {
  if (stepIndex < 0 || stepIndex >= numberOfStrainSteps) {
    abortProtocol();
    return;
  }

  targetStrainPct = strainTargetsPct[stepIndex];

  float deltaStrainPct = targetStrainPct - currentStrainPct;

  if (deltaStrainPct < 0.0) {
    abortProtocol();
    return;
  }

  float deltaMm =
    originalLengthMm * deltaStrainPct / 100.0;

  float rotations =
    deltaMm / mmPerServoRotation;

  startTimedMove(rotations, true);

  protocolState = STATE_STEP_MOVE;
  stateStartMs = millis();
}

void updateReturnToZero() {
  if (latestForceN <= zeroForceThresholdN) {
    stopServo();

    // Continue relaxing for another 3 full rotations to create slack.
    startTimedMove(extraSlackRotations, false);

    protocolState = STATE_EXTRA_SLACK;
    stateStartMs = millis();
    return;
  }

  int command = servoStop - stretchDirection * autoMoveOffsetUs;
  command = constrain(command, servoMin, servoMax);
  myServo.writeMicroseconds(command);
}


// ============================================================
// TIMED MOTION HELPERS
// ============================================================

void startTimedMove(float rotations, bool stretch) {
  if (rotations <= 0.0) {
    timedMoveActive = false;
    stopServo();
    return;
  }

  float secondsPerRotation =
    stretch ? secondsPerRotationStretch : secondsPerRotationRelax;

  float durationSec = rotations * secondsPerRotation;

  timedMoveDurationMs = (unsigned long)(durationSec * 1000.0);

  if (timedMoveDurationMs < 1) {
    timedMoveDurationMs = 1;
  }

  if (stretch) {
    timedMoveCommandUs =
      servoStop + stretchDirection * autoMoveOffsetUs;
  } else {
    timedMoveCommandUs =
      servoStop - stretchDirection * autoMoveOffsetUs;
  }

  timedMoveCommandUs =
    constrain(timedMoveCommandUs, servoMin, servoMax);

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
// SENSOR READING
// ============================================================

void readSensorsAtInterval() {
  unsigned long currentTime = millis();

  if (currentTime - previousSensorReadTime >= sensorIntervalMs) {
    previousSensorReadTime = currentTime;

    latestForceValid = false;
    latestResistanceValid = false;

    // ---------------- Load cell force ----------------

    if (scale.is_ready()) {
      float rawLoadCell = scale.get_value(loadCellSamples);
      latestForceN = rawLoadCell * newtonPerRawValue * loadDirection;
      latestForceValid = true;
    }

    // ---------------- Resistance ----------------

    float rxRawOhm = 0.0;
    float rxCalibratedOhm = 0.0;

    latestResistanceValid =
      calculateRxResistance(rxRawOhm, rxCalibratedOhm);

    if (latestResistanceValid) {
      latestRxOhm = rxCalibratedOhm;
    }

    // ---------------- MATLAB serial output ----------------

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
      Serial.print(currentStepIndex);
      Serial.print(",");
      Serial.println(targetStrainPct, 3);
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

  float bridgeSupplyCounts =
    bridgeSenseCounts * bridgeSenseMultiplier;

  if (bridgeSupplyCounts <= 0.0) {
    return false;
  }

  float vdiffCounts =
    bridgePolarity *
    (ad623OutCounts - ad623RefCounts) /
    ad623Gain;

  float nodeARatio = R3 / (R1 + R3);

  float nodeBRatio =
    nodeARatio + (vdiffCounts / bridgeSupplyCounts);

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
    case STATE_PRELOAD:
      return "PRELOAD";
    case STATE_PRELOAD_HOLD:
      return "PRELOAD_HOLD";
    case STATE_STEP_MOVE:
      return "STEP_MOVE";
    case STATE_STEP_HOLD:
      return "STEP_HOLD";
    case STATE_FINAL_HOLD:
      return "FINAL_HOLD";
    case STATE_FORCE_LIMIT_HOLD:
      return "FORCE_LIMIT_HOLD";
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
