#include <MeMCore.h>
#define SERIAL_DEBUG          // UNCOMMENT to print colou debug info (Serial Monitor 115200)

//MOTOR
MeDCMotor leftMotor(M1);  // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeLineFollower lineFinder(PORT_1);
#define motorForwardLeft -125
#define motorForwardRight 120
#define motorReverseLeft 125
#define motorReverseRight -120

#define turnLeftTime 620      
#define turnRightTime 620     
#define doubleLeftStraightTime 1005  // faster one-grid segment, estimated from old timing
#define doubleRightStraightTime 875 // faster one-grid segment, estimated from old timing
#define doubleTurnStraight 1215   
#define nudgeCorrection 30
#define motorStopTime 300
MeRGBLed led(7, 2);

//ULTRASONIC SENSOR
MeUltrasonicSensor ultrasonic(PORT_2);
#define ultrasonicMin 1   // min valid reading (filters out failed 0 readings)
#define ultrasonicMax 17  // above this = no left wall

#define tooCloseLeft 8.0
#define irTooClose -125



// BLACK LINE STATE
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
#define COLOUR_SAMPLES 1        // readings averaged per marker (3 = more stable, ~1 s slower per marker)

// COLOUR CALIBRATION
// Raw {shineRed, shineGreen, shineBlue} readings for each paper
// The code normalises these itself using BLACK_REF and the WHITE row below.
const int RAW_REF[6][3] = {
  {532, 854, 344},  // 1 Red
  {726, 729, 533},  // 2 Green
  {642, 868, 466},  // 3 Orange
  {742, 876, 629},  // 4 Pink
  {710, 705, 629},  // 5 Light blue
  {798, 870, 682}  // 6 White
};
const int BLACK_REF[3] = {374, 507, 283};  // black paper (the "0" for normalising)

#ifdef SERIAL_DEBUG
const char* NAMES[7] = {"UNKNOWN", "RED", "GREEN", "ORANGE", "PINK", "LIGHT BLUE", "WHITE"};
#endif

// DECODER
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

//BUZZER & CELEBRATION TUNE
MeBuzzer buzzer;

#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_D5  587
#define NOTE_E5  659
#define REST     0

#define beatMs 130

// 14-note sequence
const int melody[] = {
  NOTE_G4, NOTE_B4, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5, 
  NOTE_E5, NOTE_D5, NOTE_B4, NOTE_G4, NOTE_B4, NOTE_A4, NOTE_G4
};

const float beats[] = {
  1, 1, 0.5, 0.5, 0.5, 0.5, 1, 
  1, 1, 1, 1, 1, 1, 2
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

// LINE SENSOR
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

// MOTOR CODE
void stopMotor() {
  leftMotor.stop();
  rightMotor.stop();
  delay(motorStopTime);
}

void moveForward() {
  leftMotor.run(motorForwardLeft - 95);
  rightMotor.run(motorForwardRight + 135); // extra right-wheel power to counter rightward drift
}

void moveForward2Sides() {
  leftMotor.run(motorForwardLeft - 75);
  rightMotor.run(motorForwardRight + 95);
}

//to steer the robot
void moveForwardCorrected(float correction) {
  int leftSpeed  = motorForwardLeft  + (int)correction - 75;
  int rightSpeed = motorForwardRight + (int)correction + 25; // 27-32 same right-wheel trim as straight motion
  leftSpeed  = constrain(leftSpeed,  -255, 0);
  rightSpeed = constrain(rightSpeed,  0, 255);
  leftMotor.run(leftSpeed);
  rightMotor.run(rightSpeed);
}

void turnRight() {
  leftMotor.run(motorForwardLeft - 90);
  rightMotor.run(0);
  delay(turnRightTime);
  stopMotor();
}

void turnLeft() {
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight + 50);
  delay(turnLeftTime);
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
  moveForward();
  delay(doubleLeftStraightTime-50 );
  stopMotor();
  //turnLeft();
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight + 50);
  delay(turnLeftTime-50);
}

void doubleRightTurn() {
  turnRight();
  moveForward2Sides();
  delay(doubleRightStraightTime-250);
  stopMotor();
  //turnRight();
  leftMotor.run(motorForwardLeft - 100);
  rightMotor.run(motorReverseRight + 15);
  delay(turnRightTime-100);
  stopMotor();
}


// COLOUR DETECTION (NORMALISED)
long colourReading() {
  delay(ledSettleTime);
  long total = 0;
  for (int i = 0; i < 5; i++) { 
    total += analogRead(ldrPin);
    delay(5);
  }
  return total / 5;
}

// 0 = black paper, 1 = white paper.
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

  // Nearest reference colour, compared in normalised space
  int best = 1, second = 1;
  float bestDist = 100000, secondDist = 100000;
  for (int c = 0; c < 6; c++) {
    float dr = norm[0] - normaliseReading(RAW_REF[c][0], BLACK_REF[0], RAW_REF[5][0]);
    float dg = norm[1] - normaliseReading(RAW_REF[c][1], BLACK_REF[1], RAW_REF[5][1]);
    float db = norm[2] - normaliseReading(RAW_REF[c][2], BLACK_REF[2], RAW_REF[5][2]);
    float dist = sqrt(dr * dr + dg * dg + db * db);

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

// The mBOtLEDs use index 0 for the right LED and index 1 for the left LED.
// left LED: ultrasonic correction, right LED: IR correction.
void showSensorIndicators(float left, float right) {
  bool ultrasonicActive = (left >= ultrasonicMin && left < tooCloseLeft);
  bool irActive = (right < irTooClose);

  if (ultrasonicActive) led.setColorAt(0, 0, 255, 0); // left LED green
  else                  led.setColorAt(0, 0, 0, 0);

  if (irActive) led.setColorAt(1, 0, 0, 255); // right LED blue
  else          led.setColorAt(1, 0, 0, 0);

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

// SENSORS
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
}

void loop() {
  // LINE SENSOR AND COLOUR DETECTOR
  checkLine();
  if (lineSeen) {
    stopMotor();
    delay(100);
    instructions(detectColour());
    markerHandled = true;
    lineSeen = false;
    return;
  }
  if (markerHandled && !lineSensor()) {
    markerHandled = false;
  }

  // READING WALLS
  float left  = leftDistance();
  float right = rightDistance();
  if (lineSeen) return; 

  showSensorIndicators(left, right);

#ifdef SERIAL_DEBUG
  Serial.print("Left: ");  Serial.print(left);
  Serial.print(" cm | IR: "); Serial.println(right);
#endif

  // STEERING
  if (left >= ultrasonicMin && left < tooCloseLeft) {
    moveForwardCorrected(-nudgeCorrection + 30);   // too close to left: gently steer right
  } else if (right < irTooClose) {
    moveForwardCorrected(nudgeCorrection + 70);    // too close to right: gently steer left
  } else {
    moveForward();                              // clear of both walls: straight
  }
}
