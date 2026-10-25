#include <MeMCore.h>

//MOTOR
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
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

//LINE FINDER
MeLineFollower lineFinder(PORT_1); //assigning lineFinder to port 1

//ULTRASONIC SENSOR
MeUltrasonicSensor ultrasonic(PORT_2); //assigning ultrasonic to port 2
#define ultrasonicMin 1.0 //min reading for ultrasonic sensor
#define ultrasonicMax 18.0  //max reading for ultrasonic sensor

#define tooCloseLeft 7

//LED/LDR/IR 
#define DecoderA A3
#define DecoderB A2


void celebrate() {}// Code for playing celebratory tune}

/*MOTOR CODE*/
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
  delya(doubleTurnTime);
  turnRight();
}

void nudgeLeft() {}// Code for nudging slightly to the left for some short interval}
void nudgeRight() { // Code for nudging slightly to the right for some short interval}
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorNudgeSpeed);
  delay(nudgeTime);
}
/*LED/LDR/IR CODE*/
/*H
* Decoder Truth Table:
* B | A | Y0 | Y1 | Y2 | Y3 
* L   L   L    H    H    H
* L   H   H    L    H    H
* H   L   H    H    L    H
* H   H   H    H    H    L 

* Y0 is IR 
* Y1 is R
* Y2 is G
* Y3 is B

* A2 is B
* A3 is A
*/

void shineIR() {// Code for turning on the IR emitter only
    digitalWrite(DecoderA, LOW);
    digitalWrite(DecoderB, LOW);
}
void shineRed() {// Code for turning on the red LED only
    digitalWrite(DecoderA, LOW);
    digitalWrite(DecoderB, HIGH);
}
void shineGreen() {}// Code for turning on the green LED only}
void shineBlue() {}// Code for turning on the blue LED only}

float rightDistance(){ // Code for using IR circuit
  int ambient_reading = 0; 
  int reading = 0;
  shineRed(); // Turn off the IR emitter
  delay(100);
  //from here we read what the detector is getting and add it to ambient reading
  //can find the mean ambient reading also
  //then we on the IR emitter
  //read what the detector is getting and add it to reading
  //can find the mean reading 
  //find the result which is ambient - reading
  //need to scale it??
}
bool lineSensor(){ //Code for using the line finder
  return (lineFinder.readSensors() == S1_IN_S2_IN);
  //return true if detected black
}

int detectColour()
{
// Shine Red, read LDR after some delay
// Shine Green, read LDR after some delay
// Shine Blue, read LDR after some delay
// Run algorithm for colour decoding
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

void setup()
{
// Configure pinMode for A0, A1, A2, A3
pinMode(A3, OUTPUT);
digitalWrite(A3, LOW);
pinMode(A2, OUTPUT);
digitalWrite(A2, HIGH);
delay(1000);
Serial.begin(9600);
}

void loop(){

/*
moveForward();
delay(1000);
doubleLeftTurn();

/*
//for next week checkpoint
moveForward();

if (lineSensor()){
    stopMotor();
    turnLeft();
  }
  
float leftDist = leftDistance();
if (leftDist < tooCloseLeft){
  nudgeRight();
}

/*
  //checking ultrasonic distance
  bool ultrasonicIsGood = true;
  float leftDist = leftDistance();
  ultrasonicIsGood = ((ultrasonicMin < leftDist) && (ultrasonicMax > rightDist)) 
  //will be true if the reading in between min and max, means can keep going straight

  //checking ir distance
  
  //if black strip detected, stop motor, detect colour
  if (lineSensor()){
    stopMotor();
    //insert detect colour function;
    //turn correspondingly
  }

  //run the motors
  //if ultrasonic and ir reading good, then move straight
  //if ultrasonic not good, nudge right
  //if ir not good,, nudge left
  if (leftDist < tooCloseLeft){
    nudgeRight();
  } //add else if for IR here
  else {
    moveForward();
  }
*/
}
// Read ultrasonic sensing distance (choose an appropriate timeout)
// Read IR sensing distance (turn off IR, read IR detector, turn on IR, read IR detector, estimate distance)
// if within black line, stop motor, detect colour, and take corresponding action
// 	else if too near to left wall, nudge right
// 	else if too near to right wall, nudge left
// 	else move forward

// debug ultrasonic sensor
// add Serial.begin(9600); in setup
// Serial.print("Ultrasonic Reading: "); Serial.println(leftDist, 3); in loop

