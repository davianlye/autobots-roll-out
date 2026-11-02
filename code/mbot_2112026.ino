// ================== A-MAZE-ING RACE: FULL SKETCH ==================
// Colour detection uses RAW LDR readings (no normalisation).
// Wall following: PID on the left ultrasonic (set USE_PID to 0 to use the old nudge method).
// Serial Monitor: 115200 baud.

#define SERIAL_DEBUG        // comment this out for the real run to make the loop faster
#define USE_PID 1           // 1 = PID wall following, 0 = old nudgeLeft/nudgeRight method

#include <MeMCore.h>

MeDCMotor leftMotor(M1);  // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeLineFollower lineFinder(PORT_1);
#define motorStopLeft 0
#define motorStopRight 0
#define motorForwardLeft -200
#define motorForwardRight 210
#define motorReverseLeft 125
#define motorReverseRight -120
#define motorNudgeSpeed 150

#define turnLeftTime 620
#define turnRightTime 620
#define doubleTurnTime 1000
#define nudgeTime 300
#define motorStopTime 300
MeRGBLed led(7, 2);

//ULTRASONIC SENSOR
MeUltrasonicSensor ultrasonic(PORT_2); //assigning ultrasonic to port 2
#define ultrasonicMin 1.0   //min reading for ultrasonic sensor
#define ultrasonicMax 18.0  //max reading for ultrasonic sensor (above this = no left wall)

#define tooCloseLeft 7
#define irTooClose -600

// ================= PID (wall following on the LEFT ultrasonic) =================
#define targetLeft 9.0      // cm from left wall when the robot is CENTRED in the lane. Measure this!
#define Kp 15.0             // motor units per cm of error
#define Ki 0.0              // start at 0, only add if the robot sits consistently off-centre
#define Kd 2.0              // motor units per (cm/s) of error change
#define maxCorrection 80.0  // never change a wheel speed by more than this
#define maxIntegral 20.0    // anti-windup limit on the integral term (cm*s)
#define deadband 0.5        // ignore errors smaller than this (cm) to stop jitter
#define irCorrection 50     // fixed steer-left amount when only the IR sees the right wall

float pidIntegral = 0;
float pidLastError = 0;
unsigned long pidLastTime = 0;
bool pidFirstRun = true;

// ================= BLACK LINE STATE =================
bool markerHandled = false; // true while still sitting on a marker we've already acted on
bool lineSeen = false;      // true once black is detected, until the marker is handled

//LED/LDR/IR
#define DecoderA A3
#define DecoderB A2

#define irPin A0
#define irDelay 5
#define irNumReading 3
#define ledSettleTime 100
#define ldrPin A1

// ================= COLOUR CALIBRATION (RAW LDR READINGS) =================
// {reading with shineRed, reading with shineGreen, reading with shineBlue} for each paper.
// Re-take these in the demo room if lighting changes.
const int RAW_REF[6][3] = {
  {426, 863, 459},  // 1 Red
  {613, 740, 577},  // 2 Green
  {537, 863, 471},  // 3 Orange
  {626, 880, 660},  // 4 Pink
  {600, 714, 675},  // 5 Light blue
  {690, 874, 711}   // 6 White
};

const char* NAMES[7] = {"UNKNOWN", "RED", "GREEN", "ORANGE", "PINK", "LIGHT BLUE", "WHITE"};

// ---- Decoder ----
void shineIR() {  // IR emitter on, colour LEDs off
  digitalWrite(DecoderA, LOW);
  digitalWrite(DecoderB, LOW);
}
void shineRed() {
  digitalWrite(DecoderA, HIGH);
  digitalWrite(DecoderB, LOW);
}
void shineGreen() {
  digitalWrite(DecoderA, LOW);
  digitalWrite(DecoderB, HIGH);
}
void shineBlue() {
  digitalWrite(DecoderA, HIGH);
  digitalWrite(DecoderB, HIGH);
}

//BUZZER
MeBuzzer buzzer;
#define NOTE_C4   262
#define NOTE_D4   294
#define NOTE_EB4  311
#define NOTE_F4   349
#define NOTE_G4   392
#define NOTE_BB4  466
#define REST        0

#define beatMs 200   // smaller = faster

const int melody[] = {
  NOTE_G4, NOTE_F4, NOTE_G4, NOTE_BB4, NOTE_F4, NOTE_EB4, NOTE_D4,
  NOTE_F4, NOTE_F4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_C4, NOTE_BB4,
  NOTE_G4, NOTE_F4, NOTE_G4, NOTE_BB4, NOTE_F4, NOTE_EB4, NOTE_D4, NOTE_F4,
  NOTE_D4, NOTE_C4, NOTE_G4
};

const float beats[] = {
  1, 0.5, 0.5, 1, 0.5, 0.5, 1.5,
  0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.5,
  1, 0.5, 0.5, 1, 0.5, 0.5, 1, 0.5,
  0.5, 1, 2
};

const int numNotes = sizeof(melody) / sizeof(melody[0]);

void celebrate() {
  stopMotor();
  delay(200);
  for (int i = 0; i < numNotes; i++) {
    int dur = beats[i] * beatMs;
    if (melody[i] != REST) {
      buzzer.tone(melody[i], dur * 0.9);
    }
    delay(dur);
  }
  buzzer.noTone();
}

// ================= MOTOR CODE =================
void stopMotor() {
  leftMotor.stop();
  rightMotor.stop();
  delay(motorStopTime);
}

void moveForward() {
  leftMotor.run(motorForwardLeft);   // Negative: wheel turns anti-clockwise
  rightMotor.run(motorForwardRight); // Positive: wheel turns clockwise
}

// Drive forward while steering by "correction".
// correction < 0 -> steer RIGHT, correction > 0 -> steer LEFT.
void moveForwardCorrected(float correction) {
  int leftSpeed  = motorForwardLeft  + (int)correction;
  int rightSpeed = motorForwardRight + (int)correction;
  leftSpeed  = constrain(leftSpeed,  -255, 0);   // keep left wheel going forward
  rightSpeed = constrain(rightSpeed,  0, 255);   // keep right wheel going forward
  leftMotor.run(leftSpeed);
  rightMotor.run(rightSpeed);
}

void turnRight() { // 90 deg right
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorReverseRight);
  delay(turnRightTime);
  stopMotor();
}

void turnLeft() { // 90 deg left
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight);
  delay(turnLeftTime);
  stopMotor();
}

void uTurn() { // 180 deg - NOTE: currently uses the 90 deg time, tune this (likely ~2x)
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight);
  delay(turnLeftTime * 2);
  stopMotor();
}

void doubleLeftTurn() {
  turnLeft();
  moveForward();
  delay(doubleTurnTime);
  turnLeft();
}

void doubleRightTurn() {
  turnRight();
  moveForward();
  delay(doubleTurnTime);
  turnRight();
}

void nudgeLeft() { // only used when USE_PID is 0
  leftMotor.run(-motorNudgeSpeed);
  rightMotor.run(motorForwardRight);
  delay(350);
}

void nudgeRight() { // only used when USE_PID is 0
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorNudgeSpeed);
  delay(nudgeTime);
}

// ================= BLACK LINE =================
bool lineSensor() {
  return (lineFinder.readSensors() != S1_OUT_S2_OUT); // true if EITHER sensor sees black
}

// Called often so the line can't slip past between checks.
void checkLine() {
  if (!markerHandled && lineSensor()) {
    leftMotor.stop();     // brake now, no delay
    rightMotor.stop();
    lineSeen = true;
  }
}

// ================= COLOUR DETECTION (RAW) =================
long colourReading() {
  delay(ledSettleTime);   // let LED and LDR settle
  long total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(ldrPin);
    delay(5);
  }
  return total / 5;       // average of 5 readings
}

// Returns 1=red 2=green 3=orange 4=pink 5=light blue 6=white (always the closest match)
int detectColour() {
  int raw[3];
  shineRed();   raw[0] = colourReading();
  shineGreen(); raw[1] = colourReading();
  shineBlue();  raw[2] = colourReading();
  shineIR();

  int best = 1;
  float bestDist = 100000;
  for (int c = 0; c < 6; c++) {
    float dr = raw[0] - RAW_REF[c][0];
    float dg = raw[1] - RAW_REF[c][1];
    float db = raw[2] - RAW_REF[c][2];
    float dist = sqrt(dr * dr + dg * dg + db * db);
    if (dist < bestDist) {
      bestDist = dist;
      best = c + 1;
    }
  }


#ifdef SERIAL_DEBUG
  Serial.print("COLOUR RAW {"); Serial.print(raw[0]); Serial.print(", ");
  Serial.print(raw[1]); Serial.print(", "); Serial.print(raw[2]);
  Serial.print("} -> "); Serial.print(NAMES[best]);
  Serial.print(" (dist "); Serial.print(bestDist, 0); Serial.println(")");
#endif

  return best;
}

void showColour(int colour) {
  if (colour == 1)      led.setColor(0, 255, 0, 0);       // Red
  else if (colour == 2) led.setColor(0, 0, 255, 0);       // Green
  else if (colour == 3) led.setColor(0, 255, 100, 0);     // Orange
  else if (colour == 4) led.setColor(0, 255, 60, 150);    // Pink
  else if (colour == 5) led.setColor(0, 100, 200, 255);   // Light blue
  else if (colour == 6) led.setColor(0, 255, 255, 255);   // White
  else                  led.setColor(0, 0, 0, 0);
  led.show();
}

void instructions(int colour) {
  showColour(colour);   // LED stays this colour until the next marker

  if (colour == 1)      turnLeft();         // red
  else if (colour == 2) turnRight();        // green
  else if (colour == 3) uTurn();            // orange
  else if (colour == 4) doubleLeftTurn();   // pink
  else if (colour == 5) doubleRightTurn();  // light blue
  else if (colour == 6) {                   // white = end of maze
    stopMotor();
    celebrate();
    while (true) {}                         // stay stopped forever
  }
}

// ================= WALL SENSORS =================
float rightDistance() { // IR: (IR on) - (ambient). More negative = closer
  int ambientReading = 0;
  int reading = 0;
  shineRed(); // IR emitter off
  delay(irDelay);
  for (int i = 0; i < irNumReading; i++) {
    ambientReading += analogRead(irPin);
    delay(irDelay);
  }
  ambientReading /= irNumReading;
  checkLine();
  shineIR();
  delay(irDelay);
  for (int i = 0; i < irNumReading; i++) {
    reading += analogRead(irPin);
    delay(irDelay);
  }
  reading /= irNumReading;
  return reading - ambientReading;
}

float leftDistance() { // ultrasonic, median of 3
  float a = ultrasonic.distanceCm();
  checkLine();
  delay(20);
  float b = ultrasonic.distanceCm();
  checkLine();
  delay(20);
  float c = ultrasonic.distanceCm();
  checkLine();

  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

// ================= PID =================
void resetPID() {
  pidIntegral = 0;
  pidLastError = 0;
  pidFirstRun = true;
}

float computePID(float left) {
  unsigned long now = millis();
  float dt = (now - pidLastTime) / 1000.0;
  pidLastTime = now;

  float error = left - targetLeft;           // +ve = too far from wall, -ve = too close
  if (abs(error) < deadband) error = 0;

  if (pidFirstRun || dt <= 0 || dt > 0.5) {  // no valid history: P only
    pidFirstRun = false;
    pidLastError = error;
    return constrain(Kp * error, -maxCorrection, maxCorrection);
  }

  pidIntegral += error * dt;
  pidIntegral = constrain(pidIntegral, -maxIntegral, maxIntegral);

  float derivative = (error - pidLastError) / dt;
  pidLastError = error;

  float correction = Kp * error + Ki * pidIntegral + Kd * derivative;
  return constrain(correction, -maxCorrection, maxCorrection);
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  pinMode(DecoderA, OUTPUT);
  pinMode(DecoderB, OUTPUT);
  pinMode(irPin, INPUT);
  digitalWrite(DecoderA, 0);
  digitalWrite(DecoderB, 0);
  led.setColor(0, 0, 0, 0);
  led.show();
  pidLastTime = millis();
}

void loop() {
  // ---------- 1. Black line / colour marker ----------
  checkLine();
  if (lineSeen) {
    stopMotor();
    delay(100);
    instructions(detectColour());
    markerHandled = true;  // ignore this marker until we've driven off it
    lineSeen = false;
    resetPID();
    return;
  }
  if (markerHandled && !lineSensor()) {
    markerHandled = false; // both sensors see white again: ready for the next marker
  }

  // ---------- 2. Read walls (these also call checkLine() inside) ----------
  float left  = leftDistance();
  float right = rightDistance();
  if (lineSeen) return;    // line found mid-read: motors already stopped

#ifdef SERIAL_DEBUG
  Serial.print("Left: ");  Serial.print(left);
  Serial.print(" cm | IR: "); Serial.println(right);
#endif

  // ---------- 3. Steer ----------
#if USE_PID
  bool leftWallSeen = (left >= ultrasonicMin && left <= ultrasonicMax);

  if (leftWallSeen) {
    moveForwardCorrected(computePID(left));   // PID keeps us targetLeft from the left wall
  } else if (right < irTooClose) {
    resetPID();
    moveForwardCorrected(irCorrection);       // no left wall, right too close: steer left
  } else {
    resetPID();
    moveForward();                            // no left wall, right fine: straight
  }
#else
  if (left >= ultrasonicMin && left < tooCloseLeft) {
    nudgeRight();
  } else if (right < irTooClose) {
    nudgeLeft();
  } else {
    moveForward();
  }
#endif
}
