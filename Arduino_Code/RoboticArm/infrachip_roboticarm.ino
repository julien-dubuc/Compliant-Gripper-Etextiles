#include <Servo.h>

// ============================================================
// E-TEXTILE SMART GRIPPER - AUTO HOLD & RELEASE
// Arduino UNO R4 WiFi
// Output: DATA,time_ms,Rx_ohm,Delta_R_ohm
// ============================================================

const unsigned long sensorIntervalMs = 50;  // 20 Hz

const int adcBits = 14;
const int adcMaxValue = 16383;

// ============================================================
// PINS
// ============================================================
const int servoPin = 9;
const int tareButtonPin = 4;      // D4 : Tare
const int protocolButtonPin = 7;  // D7 : Cycle (Fermeture Auto -> Ouverture -> Stop)
const int ad623OutPin = A0;

// ============================================================
// SERVO & MOTOR SETTINGS
// ============================================================
Servo myServo;
int servoStop = 1520;
const int servoMin = 1000;
const int servoMax = 2000;

const int slowSpeedOffset = 100; 
const int moveDirection = 1; 

// ============================================================
// LOGIQUE DE DÉTECTION D'OBJET (SMART GRASP)
// ============================================================
// ⚠️ MODIFIE CE SEUIL SELON TES TESTS. 
// Si ça s'arrête trop tôt dans le vide, augmente la valeur (ex: 3.5).
// Si ça écrase trop les objets fragiles, diminue la valeur (ex: 1.5).
const float graspThresholdOhm = 1; 

enum SystemMode { STOPPED, CLOSING_UNTIL_CONTACT, HOLDING_OBJECT, OPENING };
SystemMode currentMode = STOPPED;

// ============================================================
// WHEATSTONE BRIDGE
// ============================================================
const float R1 = 1000.0;  
const float R2 = 1000.0;  
const float R3 = 250.0;   
const float R4 = 250.0;   
const float ad623Gain = 10.174;
const float bridgePolarity = 1.0;
const int analogSamples = 20;

const bool useResistanceCalibration = true;
const float rawRxAtShort = -58.0;
const float rawRxAtKnown = -34.0;
const float knownRxOhm = 38.8;

unsigned long previousSensorReadTime = 0;
float currentRxOhm = 0.0;
float resistanceOffset = 0.0;

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(adcBits);
  pinMode(tareButtonPin, INPUT_PULLUP);
  pinMode(protocolButtonPin, INPUT_PULLUP);

  myServo.attach(servoPin);
  myServo.writeMicroseconds(servoStop);
  
  float dummyRaw;
  if(calculateRxResistance(dummyRaw, currentRxOhm)) {
    resistanceOffset = currentRxOhm;
  }
  
  delay(1000);
  Serial.println("DATA,time_ms,Rx_ohm,Delta_R_ohm");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  handleTareButton();
  handleProtocolButton();
  updateMotor();
  readSensorsAtInterval();
}

// ============================================================
// BUTTONS LOGIC
// ============================================================
void handleTareButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastChangeTime = 0;
  bool reading = digitalRead(tareButtonPin);

  if (reading != lastReading) lastChangeTime = millis();
  if ((millis() - lastChangeTime) > 50) {
    if (reading != stableState) {
      stableState = reading;
      if (stableState == LOW) {
        resistanceOffset = currentRxOhm; 
      }
    }
  }
  lastReading = reading;
}

void handleProtocolButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastChangeTime = 0;
  bool reading = digitalRead(protocolButtonPin);

  if (reading != lastReading) lastChangeTime = millis();
  if ((millis() - lastChangeTime) > 50) {
    if (reading != stableState) {
      stableState = reading;
      if (stableState == LOW) {
        // Machine à états pour la démo
        if (currentMode == STOPPED) {
          currentMode = CLOSING_UNTIL_CONTACT; // Lance la prise
        } 
        else if (currentMode == HOLDING_OBJECT || currentMode == CLOSING_UNTIL_CONTACT) {
          currentMode = OPENING; // Force le relâchement
        } 
        else if (currentMode == OPENING) {
          currentMode = STOPPED; // Arrêt complet
        }
      }
    }
  }
  lastReading = reading;
}

// ============================================================
// SENSOR READING & AUTO-STOP LOGIC
// ============================================================
void readSensorsAtInterval() {
  unsigned long currentTime = millis();
  if (currentTime - previousSensorReadTime >= sensorIntervalMs) {
    previousSensorReadTime = currentTime;

    float rxRawOhm = 0.0;
    if (calculateRxResistance(rxRawOhm, currentRxOhm)) {
      float deltaR = currentRxOhm - resistanceOffset;
      
      // AUTO-STOP : Si on ferme et qu'on touche l'objet
      if (currentMode == CLOSING_UNTIL_CONTACT && deltaR >= graspThresholdOhm) {
        currentMode = HOLDING_OBJECT; 
      }
      
      Serial.print("DATA,");
      Serial.print(currentTime);
      Serial.print(",");
      Serial.print(currentRxOhm, 3);
      Serial.print(",");
      Serial.println(deltaR, 3);
    }
  }
}

// ============================================================
// MOTOR CONTROL
// ============================================================
void updateMotor() {
  int command = servoStop;
  
  if (currentMode == CLOSING_UNTIL_CONTACT) {
    command = servoStop + (moveDirection * slowSpeedOffset);
  } 
  else if (currentMode == OPENING) {
    command = servoStop - (moveDirection * slowSpeedOffset);
  }
  // En mode STOPPED ou HOLDING_OBJECT, command reste à servoStop

  command = constrain(command, servoMin, servoMax);
  myServo.writeMicroseconds(command);
}

// ============================================================
// WHEATSTONE CALCULATION
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

bool calculateRxResistance(float &rxRawOhm, float &rxCalibratedOhm) {
  float ad623OutCounts = readAnalogAverageCounts(ad623OutPin);
  float bridgeSupplyCounts = (float)adcMaxValue; 
  float ad623RefCounts = 0.0;

  float vdiffCounts = bridgePolarity * (ad623OutCounts - ad623RefCounts) / ad623Gain;
  float nodeARatio = R3 / (R1 + R3);
  float nodeBRatio = nodeARatio + (vdiffCounts / bridgeSupplyCounts);

  if (nodeBRatio <= 0.0 || nodeBRatio >= 1.0) return false;

  float rightBottomResistance = R2 * nodeBRatio / (1.0 - nodeBRatio);
  rxRawOhm = rightBottomResistance - R4;

  if (useResistanceCalibration) {
    rxCalibratedOhm = (rxRawOhm - rawRxAtShort) * knownRxOhm / (rawRxAtKnown - rawRxAtShort);
  } else {
    rxCalibratedOhm = rxRawOhm;
  }
  return true;
}