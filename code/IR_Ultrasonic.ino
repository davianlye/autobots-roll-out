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
void shineGreen() {
    digitalWrite(DecoderA, HIGH);
    digitalWrite(DecoderB, LOW);
}// Code for turning on the green LED only}
void shineBlue() {
    digitalWrite(DecoderA, HIGH);
    digitalWrite(DecoderB, HIGH);
}// Code for turning on the blue LED only}

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

void loop()
{

  celebrate();
  

  /*float left  = leftDistance();
  float right = rightDistance();

  Serial.print("Left: ");  Serial.print(left);
  Serial.print(" cm | IR: "); Serial.println(right);

  if (left < tooCloseLeft) {
    led.setColor(0, 255, 0, 0);
    led.show();
    nudgeRight();
    
  } else if (right < irTooClose) {
    led.setColor(0, 0, 255, 0);
    led.show();
    nudgeLeft();
    

  } else {
    led.setColor(0, 0, 0, 0); 
     led.show();

    moveForward();
  }*/
}




