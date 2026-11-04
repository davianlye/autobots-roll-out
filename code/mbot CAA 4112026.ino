// ================== A-MAZE-ING RACE: FINAL ==================
// Colour detection: NORMALISED readings (0 = black paper, 1 = white paper).
// BEFORE UPLOADING:
//   1. Check RAW_REF and BLACK_REF are fresh (from colour_test_avg.ino).
//   2. Paste your teammate's turn timings.
//   3. Choose USE_PID (0 = nudges, 1 = PID).

#define SERIAL_DEBUG          // UNCOMMENT to print colour + wall debug info (Serial Monitor 115200)

#define USE_PID 0            // 0 = nudgeLeft/nudgeRight method, 1 = PID wall following
#define COLOUR_SAMPLES 1        // readings averaged per marker (3 = more stable, ~1 s slower per marker)
#define USE_GREEN_BLUE_SPLIT 0  // 1 = ratio tie-breaker for green vs light blue
#define USE_PINK_WHITE_SPLIT 0  // 1 = ratio tie-breaker for pink vs white

#include <MeMCore.h>

MeDCMotor leftMotor(M1);  // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeLineFollower lineFinder(PORT_1);
#define motorStopLeft 0
#define motorStopRight 0
#define motorForwardLeft -125
#define motorForwardRight 120
#define motorReverseLeft 125
#define motorReverseRight -120

#define turnLeftTime 620      
#define turnRightTime 620     
#define gridTime 1000 //time to travel one grid
#define doubleTurnStraight 1215   
#define nudgeCorrection 30
#define motorStopTime 300
MeRGBLed led(7, 2);

//ULTRASONIC SENSOR
MeUltrasonicSensor ultrasonic(PORT_2);
#define ultrasonicMin 1   // min valid reading (filters out failed 0 readings)
#define ultrasonicMax 17  // above this = no left wall

#define tooCloseLeft 8.5
#define irTooClose -600

// ================= PID (wall following on the LEFT ultrasonic) =================
#define targetLeft 12.0      // cm from left wall when CENTRED in the lane
#define Kp 15.0
#define Ki 0.0
#define Kd 2.0
#define maxCorrection 80.0
#define maxIntegral 20.0
#define deadband 0.5
#define irCorrection 50

float pidIntegral = 0;
float pidLastError = 0;
unsigned long pidLastTime = 0;
bool pidFirstRun = true;

// ================= BLACK LINE STATE =================
bool markerHandled = false;
bool lineSeen = false;

//LED/LDR/IR
#define DecoderA A3
#define DecoderB A2

#define irPin A0
#define irDelay 5
#define irNumReading 3
#define ledSettleTime 100
#define ldrPin A1

// ================= COLOUR CALIBRATION =================
// Raw {shineRed, shineGreen, shineBlue} readings for each paper (paste lines from colour_test_avg).
// The code normalises these itself using BLACK_REF and the WHITE row below.
const int RAW_REF[6][3] = {
  {560, 882, 518},  // 1 Red
  {746, 770, 610},  // 2 Green
  {672, 893, 540},  // 3 Orange
  {758, 894, 702},  // 4 Pink
  {724, 737, 705},  // 5 Light blue   <-- SAME AS PINK! Re-measure light blue and paste here
  {812, 891, 751}   // 6 White (also used as the "1" for normalising)
};
const int BLACK_REF[3] = {397, 525, 340};  // black paper (the "0" for normalising)

// Ratio tie-breaker splits (use raw values; calculated automatically from RAW_REF)
const float redOrangeSplit = ((float)RAW_REF[0][0] / RAW_REF[0][1] + (float)RAW_REF[2][0] / RAW_REF[2][1]) / 2;
const float greenBlueSplit = ((float)RAW_REF[1][2] / RAW_REF[1][1] + (float)RAW_REF[4][2] / RAW_REF[4][1]) / 2;
const float pinkWhiteSplit = ((float)RAW_REF[3][0] / RAW_REF[3][1] + (float)RAW_REF[5][0] / RAW_REF[5][1]) / 2;

#ifdef SERIAL_DEBUG
const char* NAMES[7] = {"UNKNOWN", "RED", "GREEN", "ORANGE", "PINK", "LIGHT BLUE", "WHITE"};
#endif

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

#define beatMs 200

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

// ================= BLACK LINE =================
bool lineSensor() {
  return (lineFinder.readSensors() != S1_OUT_S2_OUT); // either sensor on black
}

// Brake immediately if a black line is seen
void checkLine() {
  if (!markerHandled && lineSensor()) {
    leftMotor.stop();
    rightMotor.stop();
    lineSeen = true;
  }
}

// Like delay(), but keeps checking for the black line and stops early if found
void waitCheckingLine(int ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    checkLine();
    if (lineSeen) return;
  }
}

// ================= MOTOR CODE =================
void stopMotor() {
  leftMotor.stop();
  rightMotor.stop();
  delay(motorStopTime);
}

void moveForward() {
  leftMotor.run(motorForwardLeft - 75);
  rightMotor.run(motorForwardRight + 85);
}

void moveForward2Lefts() {
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorForwardRight);
}

void moveForward2Rights() {
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorForwardRight + 5);
}

// correction < 0 -> steer RIGHT, correction > 0 -> steer LEFT
void moveForwardCorrected(float correction) {
  int leftSpeed  = motorForwardLeft  + (int)correction - 75;
  int rightSpeed = motorForwardRight + (int)correction + 85;
  leftSpeed  = constrain(leftSpeed,  -255, 0);
  rightSpeed = constrain(rightSpeed,  0, 255);
  leftMotor.run(leftSpeed);
  rightMotor.run(rightSpeed);
}

void turnRight() {
  leftMotor.run(motorForwardLeft - 52);
  rightMotor.run(motorReverseRight);
  delay(turnRightTime);
  stopMotor();
}

void turnLeft() {
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight + 50);
  delay(turnLeftTime);
  stopMotor();
}

void turnRightV2() {
  leftMotor.run(motorForwardLeft -52);
  rightMotor.run(motorReverseRight + 25);
  delay(turnRightTime);
  stopMotor();
}

void uTurn() {
  // Slight reverse
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight + 50);
  delay(doubleTurnStraight);
  stopMotor();
}

void doubleLeftTurn() {
  turnLeft();
  moveForward2Lefts();
  delay(gridTime + 700);
  stopMotor();
  turnLeft();
}

void doubleRightTurn() {
  turnRight();
  moveForward2Rights();
  delay(gridTime + 450);
  stopMotor();
  turnRightV2();
}


// ================= COLOUR DETECTION (NORMALISED) =================
long colourReading() {
  delay(ledSettleTime);
  long total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(ldrPin);
    delay(5);
  }
  return total / 5;
}

// 0 = black paper, 1 = white paper. Not clamped, so drift shows up as <0 or >1.
float normaliseReading(float reading, int blackReading, int whiteReading) {
  if (blackReading == whiteReading) return 0;
  return (reading - blackReading) / (whiteReading - blackReading);
}

// Returns 1=red 2=green 3=orange 4=pink 5=light blue 6=white
int detectColour() {
  long sum[3] = {0, 0, 0};
  for (int n = 0; n < COLOUR_SAMPLES; n++) {
    shineRed();   sum[0] += colourReading();
    shineGreen(); sum[1] += colourReading();
    shineBlue();  sum[2] += colourReading();
  }
  shineIR();

  float raw[3], norm[3];
  for (int k = 0; k < 3; k++) {
    raw[k]  = (float)sum[k] / COLOUR_SAMPLES;
    norm[k] = normaliseReading(raw[k], BLACK_REF[k], RAW_REF[5][k]);
  }

#ifdef SERIAL_DEBUG
  Serial.println("========== MARKER ==========");
  Serial.print("RAW  {"); Serial.print(raw[0], 0); Serial.print(", ");
  Serial.print(raw[1], 0); Serial.print(", "); Serial.print(raw[2], 0); Serial.println("}");
  Serial.print("NORM {"); Serial.print(norm[0], 2); Serial.print(", ");
  Serial.print(norm[1], 2); Serial.print(", "); Serial.print(norm[2], 2); Serial.println("}");
  Serial.print("DIST ");
#endif

  // Nearest reference colour, compared in normalised space
  int best = 1, second = 1;
  float bestDist = 100000, secondDist = 100000;
  for (int c = 0; c < 6; c++) {
    float dr = norm[0] - normaliseReading(RAW_REF[c][0], BLACK_REF[0], RAW_REF[5][0]);
    float dg = norm[1] - normaliseReading(RAW_REF[c][1], BLACK_REF[1], RAW_REF[5][1]);
    float db = norm[2] - normaliseReading(RAW_REF[c][2], BLACK_REF[2], RAW_REF[5][2]);
    float dist = sqrt(dr * dr + dg * dg + db * db);

#ifdef SERIAL_DEBUG
    Serial.print(NAMES[c + 1]); Serial.print(":"); Serial.print(dist, 2); Serial.print("  ");
#endif
    if (dist < bestDist) {
      second = best; secondDist = bestDist;
      best = c + 1;  bestDist = dist;
    } else if (dist < secondDist) {
      second = c + 1; secondDist = dist;
    }
  }

#ifdef SERIAL_DEBUG
  Serial.println();
  Serial.print("Nearest: "); Serial.print(NAMES[best]);
  Serial.print(" ("); Serial.print(bestDist, 2); Serial.print("), runner-up: ");
  Serial.print(NAMES[second]); Serial.print(" ("); Serial.print(secondDist, 2); Serial.println(")");
  int nearest = best;
#endif

  // Ratio tie-breakers (raw ratios cancel out overall brightness changes)
  if (best == 1 || best == 3) {
    best = (raw[0] / raw[1] < redOrangeSplit) ? 1 : 3;   // below split = red
#ifdef SERIAL_DEBUG
    Serial.print("Red/orange ratio: "); Serial.print(raw[0] / raw[1], 3);
    Serial.print("  split: "); Serial.println(redOrangeSplit, 3);
#endif
  }
#if USE_GREEN_BLUE_SPLIT
  if (best == 2 || best == 5) {
    best = (raw[2] / raw[1] < greenBlueSplit) ? 2 : 5;   // below split = green
#ifdef SERIAL_DEBUG
    Serial.print("Green/blue ratio: "); Serial.print(raw[2] / raw[1], 3);
    Serial.print("  split: "); Serial.println(greenBlueSplit, 3);
#endif
  }
#endif
#if USE_PINK_WHITE_SPLIT
  if (best == 4 || best == 6) {
    best = (raw[0] / raw[1] < pinkWhiteSplit) ? 4 : 6;   // below split = pink
#ifdef SERIAL_DEBUG
    Serial.print("Pink/white ratio: "); Serial.print(raw[0] / raw[1], 3);
    Serial.print("  split: "); Serial.println(pinkWhiteSplit, 3);
#endif
  }
#endif

#ifdef SERIAL_DEBUG
  if (best != nearest) {
    Serial.print("Tie-breaker changed "); Serial.print(NAMES[nearest]);
    Serial.print(" -> "); Serial.println(NAMES[best]);
  }
  Serial.print(">>> FINAL: "); Serial.println(NAMES[best]);
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
  showColour(colour);

  if (colour == 1)      turnLeft();         // red
  else if (colour == 2) turnRight();        // green
  else if (colour == 3) uTurn();            // orange
  else if (colour == 4) doubleLeftTurn();   // pink
  else if (colour == 5) doubleRightTurn();  // light blue
  else if (colour == 6) {                   // white = end
    stopMotor();
    celebrate();
    while (true) {}
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

  float error = left - targetLeft;
  if (abs(error) < deadband) error = 0;

  if (pidFirstRun || dt <= 0 || dt > 0.5) {
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
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println("Debug on.");
#endif
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
    markerHandled = true;
    lineSeen = false;
    resetPID();
    return;
  }
  if (markerHandled && !lineSensor()) {
    markerHandled = false;
  }

  // ---------- 2. Read walls ----------
  float left  = leftDistance();
  float right = rightDistance();
  if (lineSeen) return;

#ifdef SERIAL_DEBUG
  Serial.print("Left: ");  Serial.print(left);
  Serial.print(" cm | IR: "); Serial.println(right);
#endif

  // ---------- 3. Steer ----------
#if USE_PID
  bool leftWallSeen = (left >= ultrasonicMin && left <= ultrasonicMax);

  if (leftWallSeen) {
    moveForwardCorrected(computePID(left));
  } else if (right < irTooClose) {
    resetPID();
    moveForwardCorrected(irCorrection);
  } else {
    resetPID();
    moveForward();
  }
#else
  if (left >= ultrasonicMin && left < tooCloseLeft) {
    moveForwardCorrected(-nudgeCorrection + 10);   // too close to left: gently steer right
  } else if (right < irTooClose) {
    moveForwardCorrected(nudgeCorrection);    // too close to right: gently steer left
  } else {
    moveForward();                            // clear of both walls: straight
  }
#endif
}
