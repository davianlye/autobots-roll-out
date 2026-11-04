#include <MeMCore.h>

MeDCMotor leftMotor(M1);
MeDCMotor rightMotor(M2);

// Motor speeds from your robot code
#define motorForwardLeft  -125
#define motorForwardRight  120
#define motorReverseLeft   125
#define motorReverseRight -120

// Starting timings from your code; adjust these while calibrating
#define turnLeftTime    620
#define turnRightTime   620
#define gridTime       1000   // time to travel one grid
#define motorStopTime   300
#define DoubleTurnStraight 1215

// Choose one test:
// 1 = left turn
// 2 = right turn
// 3 = U-turn
// 4 = two left turns, one grid apart
// 5 = two right turns, one grid apart
#define TEST_TURN 5

void stopMotor() {
  leftMotor.stop();
  rightMotor.stop();
  delay(motorStopTime);
}

void moveForward() {
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorForwardRight);
}

void moveForward2Rights() {
  leftMotor.run(motorForwardLeft);
  rightMotor.run(motorForwardRight + 5);
}

void turnLeft() {
  leftMotor.run(motorReverseLeft);
  rightMotor.run(motorForwardRight + 50);
  delay(turnLeftTime);
  stopMotor();
}

void turnRight() {
  leftMotor.run(motorForwardLeft - 52);
  rightMotor.run(motorReverseRight);
  delay(turnRightTime);
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
  delay(DoubleTurnStraight);
  stopMotor();
}

void twoLeftTurns() {
  turnLeft();
  moveForward();
  delay(gridTime + 700);
  stopMotor();
  turnLeft();
}

void twoRightTurns() {
  turnRight();
  moveForward2Rights();
  delay(gridTime + 450);
  stopMotor();
  turnRightV2();
}

void setup() {
  if (TEST_TURN == 1) {
    turnLeft();
  } else if (TEST_TURN == 2) {
    turnRight();
  } else if (TEST_TURN == 3) {
    uTurn();
  } else if (TEST_TURN == 4) {
    twoLeftTurns();
  } else if (TEST_TURN == 5) {
    twoRightTurns();
  }

  stopMotor();  // Stop after the selected test
}

void loop() {
  // Empty: the selected test runs only once in setup()
}
