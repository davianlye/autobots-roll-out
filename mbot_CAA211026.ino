#include <MeMCore.h>
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeLineFollower lineFinder(PORT_1); //assigning lineFinder to port 1

#define MotorStopLeft 0
#define MotorStopRight 0
#define MotorForwardLeft -200
#define MotorForwardRight 205
#define MotorReverseLeft 125
#define MotorReverseRight -120

#define TurnLeftTime 640
#define TurnRightTime 620
#define MotorStopTime 300



void celebrate() {}// Code for playing celebratory tune}

/*MOTOR CODE START*/
void stopMotor() {// Code for stopping motor
leftMotor.stop(); // Stop left motor
rightMotor.stop(); // Stop right motor
delay(MotorStopTime);
}

void moveForward() {// Code for moving forward for some short interval
  // Going forward:
leftMotor.run(MotorForwardLeft); // Negative: wheel turns anti-clockwise
rightMotor.run(MotorForwardRight); // Positive: wheel turns clockwise
}

void turnRight() { // Code for turning right 90 deg
    leftMotor.run(MotorForwardLeft);
    rightMotor.run(MotorReverseRight);
    delay(TurnRightTime);
    stopMotor();
}

void turnLeft() { // Code for turning left 90 deg
    leftMotor.run(MotorReverseLeft);
    rightMotor.run(MotorForwardRight);
    delay(TurnLeftTime);
    stopMotor();
}

void uTurn() {}// Code for u-turn}
void doubleLeftTurn() {}// Code for double left turn}
void doubleRightTurn() {}// Code for double right turn}
void nudgeLeft() {}// Code for nudging slightly to the left for some short interval}
void nudgeRight() {}// Code for nudging slightly to the right for some short interval}

/*MOTOR CODE STOP*/

bool LineSensor(){
  if(lineFinder.readSensors() == S1_IN_S2_IN){ //both sensor detect black
  Serial.println("BLACK");
    return true;
  }
  else {
    return false;
  }
}

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

void setup()
{
// Configure pinMode for A0, A1, A2, A3
delay(1000);
}

void loop(){
  while(!LineSensor()){
    moveForward();
  }
  stopMotor();
  turnLeft();
}
// Read ultrasonic sensing distance (choose an appropriate timeout)
// Read IR sensing distance (turn off IR, read IR detector, turn on IR, read IR detector, estimate distance)
// if within black line, stop motor, detect colour, and take corresponding action
// 	else if too near to left wall, nudge right
// 	else if too near to right wall, nudge left
// 	else move forward



