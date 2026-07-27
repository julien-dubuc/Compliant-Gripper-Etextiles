#include <Servo.h>
#include "HX711.h"

Servo myServo;
HX711 scale;

// --- PINS ---
const int servoPin = 9;
const int joystickPin = A0;
const int DOUT = 3;
const int CLK = 4;

// --- SERVO SETTINGS ---
int servoStop = 1520; 
const int servoMin = 1000;
const int servoMax = 2000;
const int CLOSE_SPEED = servoMax; // Vitesse pour fermer
const int OPEN_SPEED = servoMin;  // Vitesse pour ouvrir

// --- MODES & TEST ---
bool manualMode = true;
int testState = 0; // 0: Idle, 1: Fermeture (Test), 2: Ouverture (Fin test)
unsigned long testStartTime = 0;
const unsigned long TEST_DURATION = 6000; // 5 secondes de test

// --- JOYSTICK ---
int joystickCenter = 512;
const int deadband = 80;

// --- FORCE ---
float currentForce = 0.0;
float maxForce = 0.0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600);

  // Calibration Joystick
  long sum = 0;
  for (int i = 0; i < 100; i++) { sum += analogRead(joystickPin); delay(5); }
  joystickCenter = sum / 100;

  // Init Balance
  scale.begin(DOUT, CLK);
  scale.set_scale(11400.f);
  scale.tare();

  Serial.println("\n=== SYSTEME PRET ===");
  Serial.println("Commandes: 'f' (Test 5s), 'o' (Open), 'd' (Detach), 'm' (Joystick), 't' (Tare)");
}

void loop() {
  // 1. GESTION DES COMMANDES SERIAL
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'f') { // TEST AUTO
      manualMode = false;
      if (!myServo.attached()) myServo.attach(servoPin);
      myServo.writeMicroseconds(CLOSE_SPEED);
      testStartTime = millis();
      testState = 1;
      maxForce = 0;
      Serial.println("\n[TEST] Fermeture forcee, mesure en cours...");
    }
    else if (command == 'o') { manualMode = false; if(!myServo.attached()) myServo.attach(servoPin); myServo.writeMicroseconds(OPEN_SPEED); }
    else if (command == 'd') { manualMode = false; myServo.detach(); Serial.println("Servo Detached."); }
    else if (command == 'm') { manualMode = true; Serial.println("Joystick Mode."); }
    else if (command == 't') { scale.tare(); maxForce = 0; Serial.println("Tare effectuee."); }
  }

  // 2. LOGIQUE DU TEST AUTO (Non bloquant)
  if (testState == 1) { // Phase de fermeture (5s)
    if (millis() - testStartTime >= TEST_DURATION) {
      myServo.writeMicroseconds(OPEN_SPEED);
      testState = 2;
      Serial.println("[TEST] Fin 5s. Ouverture...");
      Serial.print("[RESULTAT] Force Max : "); Serial.println(maxForce);
    }
  } else if (testState == 2) { // Phase de retour
    if (millis() - testStartTime >= TEST_DURATION + 2000) {
      myServo.writeMicroseconds(servoStop);
      testState = 0;
      manualMode = true;
      Serial.println("[TEST] Termine.");
    }
  }

  // 3. MODE MANUEL
  if (manualMode) {
    int val = analogRead(joystickPin) - joystickCenter;
    if (abs(val) < deadband) { if(myServo.attached()) myServo.detach(); }
    else {
      if (!myServo.attached()) myServo.attach(servoPin);
      myServo.writeMicroseconds(val > 0 ? map(val, deadband, 512, servoStop, servoMax) : map(val, -deadband, -512, servoStop, servoMin));
    }
  }

  // 4. LECTURE FORCE
  if (scale.is_ready()) {
    currentForce = scale.get_units(1);
    if (currentForce > maxForce) maxForce = currentForce;
  }

  // 5. AFFICHAGE
  if (millis() - lastPrintTime >= 100) {
    Serial.print("Force: "); Serial.print(currentForce, 2);
    Serial.print(" N | Max: "); Serial.println(maxForce, 2);
    lastPrintTime = millis();
  }
}