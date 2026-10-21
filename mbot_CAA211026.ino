#include <MeMCore.h>
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeLineFollower lineFinder(PORT_1); //assigning lineFinder to port 1

MeUltrasonicSensor ultrasonic(PORT_2); //assigning ultrasonic to port 2
#define ULTRASONIC 12
double const tooCloseLeft = 5;


#define motorStopLeft 0
#define motorStopRight 0
#define motorForwardLeft -200
#define motorForwardRight 205
#define motorReverseLeft 125
#define motorReverseRight -120
#define motorNudgeSpeed 150

#define turnLeftTime 640
#define turnRightTime 620
#define nudgeTime 100
#define motorStopTime 300



void celebrate() {}// Code for playing celebratory tune}

/*MOTOR CODE START*/
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

void uTurn() {}// Code for u-turn}
void doubleLeftTurn() {}// Code for double left turn}
void doubleRightTurn() {}// Code for double right turn}
void nudgeLeft() {}// Code for nudging slightly to the left for some short interval}
void nudgeRight() { // Code for nudging slightly to the right for some short interval}
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorNudgeSpeed);
  delay(nudgeTime);
  stopMotor();
}

/*MOTOR CODE STOP*/


void shineIR() {}// Code for turning on the IR emitter only}
void shineRed() {}// Code for turning on the red LED only}
void shineGreen() {}// Code for turning on the green LED only}
void shineBlue() {}// Code for turning on the blue LED only}

int detectColour()
{
// Shine Red, read LDR after some delay
// Shine Green, read LDR after some delay
// Shine Blue, read LDR after some delay
// Run algorithm for colour decoding
}

float leftDistance(){
  digitalWrite(ULTRASONIC, HIGH);

  float distance = ultrasonic.distanceCm();

  double a = ultrasonic.distanceCm();
  delay(20);
  double b = ultrasonic.distanceCm();
  delay(20);
  double c = ultrasonic.distanceCm();

  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b; 
  return c;
  //returns average
}

void setup()
{
// Configure pinMode for A0, A1, A2, A3
pinMode(A3, OUTPUT);
digitalWrite(A3, LOW);
pinMode(A2, OUTPUT);
digitalWrite(A2, HIGH);
delay(1000);
}

void loop(){
  int SensorState = lineFinder.readSensors();
  bool BlackStrip = (SensorState == S1_IN_S2_IN);
  if (BlackStrip){
    stopMotor();
    delay(100);
    //detect the colour
    turnLeft();
  } else {
  double leftDist = leftDistance();
  if (leftDist < tooCloseLeft){
    nudgeRight();
  } //add else if for IR here
  else {
    moveForward();
  }
  }

}
// Read ultrasonic sensing distance (choose an appropriate timeout)
// Read IR sensing distance (turn off IR, read IR detector, turn on IR, read IR detector, estimate distance)
// if within black line, stop motor, detect colour, and take corresponding action
// 	else if too near to left wall, nudge right
// 	else if too near to right wall, nudge left
// 	else move forward



