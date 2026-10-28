#include <MeMCore.h>

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
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
#define ultrasonicMin 1.0 //min reading for ultrasonic sensor
#define ultrasonicMax 18.0  //max reading for ultrasonic sensor

#define tooCloseLeft 7
#define irTooClose -600

//LED/LDR/IR 
#define DecoderA A3
#define DecoderB A2

#define irPin A0
#define irDelay 5
#define irNumReading 3
#define ledSettleTime 100
#define ldrPin A1

// Calibration: index 0 = R, 1 = G, 2 = B (fill these in during calibration)
int white_arr[3] = {968, 995, 982};
int black_arr[3] = {916, 940, 875};

// Reference normalised (R,G,B) for each paper, in the same order as the colour numbers.
// These are STARTING GUESSES. Replace them with the values you see in Serial for each paper.

const float REF[6][3] = {
  {0.70, 0.24, 0.42},  // Red
  {0.34, 0.69, 0.69},  // Green
  {0.73, 0.47, 0.47},  // Orange
  {0.75, 0.69, 0.82},  // Pink
  {0.22, 0.58, 0.82},  // Light blue
  {0.73, 0.77, 0.89}  // White
};
const float MAX_DISTANCE = 0.45;  // bigger than this = "unknown"

// ---- Decoder: (A2, A3) LOW,HIGH = green | HIGH,HIGH = blue | HIGH,LOW = red ----
void shineIR() {// Code for turning on the IR emitter only
    digitalWrite(DecoderA, LOW);
    digitalWrite(DecoderB, LOW);
}
void shineRed()   { 
  digitalWrite(DecoderA, HIGH); 
  digitalWrite(DecoderB, LOW); 
   }
void shineGreen() { 
  digitalWrite(DecoderA, LOW);  
  digitalWrite(DecoderB, HIGH); 
  }

void shineBlue()  { 
  digitalWrite(DecoderA, HIGH); 
  digitalWrite(DecoderB, HIGH); 
  }
MeBuzzer buzzer;
#define NOTE_C4   262
#define NOTE_EB4  311
#define beatMs 214   // 60000 / 140 / 2

const int notes[] = {NOTE_EB4, NOTE_EB4, NOTE_C4, NOTE_C4,
                       NOTE_EB4, NOTE_EB4, NOTE_C4, NOTE_C4};

void celebrate() {
  stopMotor();
  delay(200);
  for (int bar = 0; bar < 4; bar++) {
    for (int i = 0; i < 8; i++) {
      buzzer.tone(notes[i], beatMs * 0.9);
      delay(beatMs);
    }
  }
  buzzer.noTone();
}
  void stopMotor() {// Code for stopping motor
  leftMotor.stop(); // Stop left motor
  rightMotor.stop(); // Stop right motor
  delay(motorStopTime);
}

void moveForward() {// Code for moving forward for some short interval
  // Going forward:
 leftMotor.run(motorForwardLeft); // Negative: wheel turns anti-clockwise
 rightMotor.run(motorForwardRight); // Positive: wheel turns clockwise
}

void turnRight() { // Code for turning right 90 deg
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorReverseRight);
  delay(turnRightTime);
  stopMotor();
}

void turnLeft() { // Code for turning left 90 deg
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight);
  delay(turnLeftTime);
  stopMotor();
}

void uTurn() { // Code for u-turn
  //slight reverse
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorReverseRight);
  delay(100);
  stopMotor();

  //slight turn towards left
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight);
  delay(turnLeftTime/2);
  stopMotor();

  //slight reverse to make turn
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorReverseRight);
  delay(200);
  stopMotor();

  //complete turn from earlier
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight);
  delay(turnLeftTime + 150); //timing to calibrate 
  stopMotor();
}

void doubleLeftTurn() { // Code for double left turn
 turnLeft();
 moveForward();
 delay(doubleTurnTime);
 turnLeft();
}

void doubleRightTurn() {// Code for double right turn
  turnRight();
  moveForward();
  delay(doubleTurnTime);
  turnRight();
}

void nudgeLeft() { // Code for nudging slightly to the left for some short interval
  leftMotor.run(-motorNudgeSpeed);
  rightMotor.run(motorForwardRight);
  delay(350);
}

void nudgeRight() { // Code for nudging slightly to the right for some short interval}
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorNudgeSpeed);
  delay(nudgeTime);
}

long colourReading() {
  delay(ledSettleTime);                 // let LED and LDR settle
  long total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(ldrPin);
    delay(5);
  }
  return total / 5;                     // average of 5 readings
}

float normaliseReading(int reading, int blackReading, int whiteReading) {
  if (blackReading == whiteReading) return 0;
  float value = (float)(reading - blackReading) / (whiteReading - blackReading);
  if (value < 0) value = 0;
  else if (value > 1) value = 1;
  return value;
}

// Returns 1=red 2=green 3=orange 4=pink 5=light blue 6=white, 0=unknown
int detectColour() {
  shineRed();  
  int redReading   = colourReading();
  shineGreen(); 
  int greenReading = colourReading();
  shineBlue(); 
  int blueReading  = colourReading();

  float rgb[3];
  rgb[0] = normaliseReading(redReading,   black_arr[0], white_arr[0]);
  rgb[1] = normaliseReading(greenReading, black_arr[1], white_arr[1]);
  rgb[2] = normaliseReading(blueReading,  black_arr[2], white_arr[2]);

#ifdef SERIAL_DEBUG
  Serial.print("R: ");  Serial.print(rgb[0]);
  Serial.print("  G: "); Serial.print(rgb[1]);
  Serial.print("  B: "); Serial.println(rgb[2]);
#endif

  // Pick the reference colour closest to what we measured
  int best = 0;
  float bestDist = 1000;
  for (int c = 0; c < 6; c++) {
    float dr = rgb[0] - REF[c][0];
    float dg = rgb[1] - REF[c][1];
    float db = rgb[2] - REF[c][2];
    float dist = sqrt(dr * dr + dg * dg + db * db);
    if (dist < bestDist) {
      bestDist = dist;
      best = c + 1;
    }
  }
  return (bestDist <= MAX_DISTANCE) ? best : 0;
}

void showColour(int colour) {
  if (colour == 1)      led.setColor(0, 255, 0, 0);       // Red
  else if (colour == 2) led.setColor(0, 0, 255, 0);       // Green
  else if (colour == 3) led.setColor(0, 255, 100, 0);     // Orange
  else if (colour == 4) led.setColor(0, 255, 60, 150);    // Pink
  else if (colour == 5) led.setColor(0, 100, 200, 255);   // Light blue
  else if (colour == 6) led.setColor(0, 255, 255, 255);   // White
  else {
    led.setColor(0, 0, 0, 0);
    Serial.println("Error");
  }
  led.show();
}

void instructions(int colour) {
 
  showColour(colour);

  if (colour == 1)      turnLeft();         // red
  else if (colour == 2) turnRight();        // green
  else if (colour == 3) uTurn();            // orange
  else if (colour == 4) doubleLeftTurn();   // pink
  else if (colour == 5) doubleRightTurn();  // light blue
  else if (colour == 6) { stopMotor(); celebrate(); }  // white
  else Serial.println("Error");
}
float rightDistance(){ // Code for using IR circuit
  int ambientReading = 0; 
  int reading = 0;
  shineRed(); // Turn off the IR emitter
  delay(irDelay);
  for (int i = 0; i < irNumReading; i++){
    ambientReading += analogRead(irPin);
    delay(irDelay);
  }
  ambientReading /= irNumReading;
  shineIR();
  delay(irDelay);
  for (int i = 0; i < irNumReading; i++){
    reading += analogRead(irPin);
    delay(irDelay);
  }
  reading /= irNumReading;
  return reading - ambientReading;


}
float leftDistance(){ //Code for using the ultrasonic sensor
  float a = ultrasonic.distanceCm();
  delay(20);
  float b = ultrasonic.distanceCm();
  delay(20);
  float c = ultrasonic.distanceCm();

  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b; 
  return c;
  //returns median
}
bool markerHandled = false;
void setup()
{
// Configure pinMode for A0, A1, A2, A3
Serial.begin(9600); // intialise serial comunication
pinMode (DecoderA, OUTPUT);// set decoder A as output
pinMode (DecoderB, OUTPUT);// set decorder B as output
pinMode (irPin, INPUT);// set ir receiver pin as input
digitalWrite(DecoderA, 0);//initialise decoder A to 0
digitalWrite(DecoderB, 0);// initialise decoder B to 0

}

void loop(){  int sensorState =lineFinder.readSensors();
  bool onBlackMarker = (sensorState == S1_IN_S2_IN);
  if (onBlackMarker && !markerHandled){
    stopMotor();
    delay(100);
    instructions(detectColour());
    markerHandled = true;
  } else if (!onBlackMarker){
    markerHandled = false;
  }
  float left  = leftDistance();
  float right = rightDistance();

  Serial.print("Left: ");  Serial.print(left);
  Serial.print(" cm | IR: "); Serial.println(right);

  if (left < tooCloseLeft) {
    led.setColor(128, 0, 255, 0);
    led.show();
    nudgeRight();
    
  } else if (right < irTooClose) {
    led.setColor(255, 255, 0, 0);
    led.show();
    nudgeLeft();
    

  } else {
    led.setColor(0, 0, 0, 0); 
     led.show();

    moveForward();
}


}