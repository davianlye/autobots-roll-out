#include <MeMCore.h>

MeDCMotor leftMotor(M1); //assigning left motor to port M1
MeDCMotor rightMotor(M2);// assigning right motor to port M2
MeBuzzer buzzer; //create the buzzer
MeRGBLed led(0,30); // based on hardware connections on mCore
MeUltrasonicSensor ultrasonic(PORT_2);
MeLineFollower lineFinder(PORT_1);

#define SERIAL_DEBUG
#define ULTRASONIC 12
#define LIGHTSENSOR A6
#define IR_PIN A0
#define LDR_PIN A1
#define DecoderA A3
#define DecoderB A2


int MOTOR_SPEED = 200  ;
int NUDGE_SPEED = 120;
int FORWARD_TIME = 700;
int TURN_TIMERight = 500;
int TURN_TIMELeft = 500;
int UTURN_TIME = 1000;
int NUDGE_TIME = 100;

int LED_SETTLE_MS = 100;
int IR_SETTLE_MS  = 5;

const double TOO_CLOSE_LEFT_CM = ;
const int    IR_TOO_CLOSE      = ;

long black_arr[] = { , , }; //based on lab reading for black paper
long white_arr[] = {, ,}; //based on lab reading for black paper

const float REF[6][3] = {
  {0, 0, 0.},   // 1 red
  {0.30, 0.80, 0.40},   // 2 green
  {0.95, 0.50, 0.20},   // 3 orange
  {0.90, 0.50, 0.60},   // 4 pink
  {0.40, 0.70, 0.90},   // 5 light blue
  {1.00, 1.00, 1.00}    // 6 white
};

void stopMotor() {// Code for stopping motor
  leftMotor.stop();
  rightMotor.stop();
}

void celebrate() {// Code for playing celebratory tune}
  stopMotor();
  int notes[] = {523, 659, 784, 1047};
  for (int i = 0; i < 4; i++) { 
    buzzer.tone(notes[i], 200);
    delay(250); 
    }
  buzzer.noTone();

}

void moveForward() {// Code for moving forward for some short interval}
  leftMotor.run(-MOTOR_SPEED);
  rightMotor.run(MOTOR_SPEED);
  
  }
void turnRight() {// Code for turning right 90 deg}
  leftMotor.run(-MOTOR_SPEED);
  rightMotor.run(-MOTOR_SPEED);
  delay(TURN_TIMERight);
  stopMotor();

}
void turnLeft() {// Code for turning left 90 deg}
  leftMotor.run(MOTOR_SPEED);
  rightMotor.run(MOTOR_SPEED);
  delay(TURN_TIMELeft);
  stopMotor();
}
void uTurn() {// Code for u-turn}
  leftMotor.run(MOTOR_SPEED);
  rightMotor.run(MOTOR_SPEED);
  delay(UTURN_TIME);
  stopMotor();
}
void doubleLeftTurn() {// Code for double left turn}
  turnLeft();
  moveForward();
  delay(FORWARD_TIME);
  stopMotor();
  turnLeft();
}
void doubleRightTurn() {// Code for double right turn}
  turnRight();
  moveForward();
  delay(FORWARD_TIME);
  stopMotor();
  turnRight();
}
void nudgeLeft() {// Code for nudging slightly to the left for some short interval}
  leftMotor.run(-NUDGE_SPEED);
  rightMotor.run(MOTOR_SPEED);
  delay(NUDGE_TIME);
  stopMotor();

}
void nudgeRight() {// Code for nudging slightly to the right for some short interval}
  leftMotor.run(-MOTOR_SPEED);
  rightMotor.run(NUDGE_SPEED);
  delay(NUDGE_TIME);
  stopMotor();
}

void shineIR() {// Code for turning on the IR emitter only}
  digitalWrite(DecoderA, LOW);
  digitalWrite(DecoderB, LOW);
}

int ir_read(){
// 1. Ambient: emitter off
  selectDecoder(HIGH,HIGH);
  delay(IR_SETTLE_MS);
  int ambient = analogRead(IR_PIN);

  // 2. Emitter on
  shineIR();
  delay(IR_SETTLE_MS);
  int withIR = analogRead(IR_PIN);

  // 3. Turn emitter off again
  selectDecoder(HIGH,HIGH);

  return abs(ambient - withIR);  

}

void shineRed() {// Code for turning on the red LED only}
  digitalWrite(DecoderA, LOW);
  digitalWrite(DecoderB, HIGH;
}
void shineGreen() {// Code for turning on the green LED only}
  digitalWrite(DecoderA, HIGH);
  digitalWrite(DecoderB, LOW);
}
void shineBlue() {// Code for turning on the blue LED only}
  digitalWrite(DecoderA, HIGH);
  digitalWrite(DecoderB, HIGH);
}

long colour_reading(){
  delay(LED_SETTLE_MS); // let the led and ldr settle

  long total=0;
  for(int i=0;i<5;i++){ 
    total += analogRead(LDR_PIN); 
    delay(5); //lets the ldr settle before sampling
  }

  return total / 5; // takes the average of 5 readings

}
 float normaliseReading (int readings,int blackReading,int whiteReading){
  if (blackReading == whiteReading){

    return 0;
  } 
  float value = (float)(reading -blackReading)/(whiteReading - blackReading);

  if (value<0){
    value= 0;
  } else  if (value>1){
    value = 1;
  }
  return value;
 }

int detectColour()
{
// Shine Red, read LDR after some delay
// Shine Green, read LDR after some delay
// Shine Blue, read LDR after some delay
// Run algorithm for colour decoding

  int redReading,greenReading,blueReading;

  shineRed();
  delay();
  redReading = colour_reading();

  shineBlue();
  delay();
  blueReading = colour_reading();

  shineGreen ();
  delay();
  greenReading = colour_reading();

  float red = normaliseReading(redReading,black_arr[0],white_arr[0]);
  float blue = normaliseReading(blueReading,black_arr[2],white_arr[2]);
  float green = normaliseReading(greenReading,black_arr[1],white_arr[1]);
  
  #ifdef SERIAL_DEBUG
    Serial.print("Red: ");
    Serial.println(red);
    Serial.print("Green: ");
    Serial.println(green);
    Serial.print("Blue: ");
    Serial.println(blue);
  #endif

  // code for returning detected colour
  if ()





}

void instructions(){
  int Colour=detectColour;
  if (Colour ==1){// red paper : left turn
    turnLeft();
  } else if (Colour==2){// green paper : right turn
    turnRight();
  } else if (Colour==3){// orange paper:uturn
    uTurn();
  } else if (Colour==4){// pink paper: double left turn
    doubleLeftTurn();
  } else if (Colour==5){// light blue paper : double right turn
    doubleRightTurn();
  } else if (Colour==6){// white paper
    stopMotor();
    celebrate();
  } else {
    Serial.println("Error");
  }
}

float leftdistance(){
  digitalWrite(ULTRASONIC_PIN, LOW);

  float distance = ultrasonic.distanceCm();

  double a = ultrasonic.distanceCm();
  delay(20);
  double b = ultrasonic.distanceCm();
  delay(20);
  double c = ultrasonic.distanceCm();

  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}


void setup()
{
// Configure pinMode for A0, A1, A2, A3
serial.begin(9600); // intialise serial comunication
pinMode (DECODER_A, OUTPUT);// set decoder A as output
pinMode (DECODER_B, OUTPUT);// set decorder B as output
pinMode (IR_RECEIVER_PIN, INPUT);// set ir receiver pin as input
pinMode (LDR_PIN, INPUT);// set ldr pin as input
led.setpin(13);// set rgb led control pin 
analogwrite(DECODER_A, 0);//initialise decoder A to 0
analogwrite(DECODER_B, 0);// initialise decoder B to 0

bool markerHandled = false;


}

void loop()
{
// Read ultrasonic sensing distance (choose an appropriate timeout)
// Read IR sensing distance (turn off IR, read IR detector, turn on IR, read IR detector, estimate distance)
// if within black line, stop motor, detect colour, and take corresponding action
// 	else if too near to left wall, nudge right
// 	else if too near to right wall, nudge left
// 	else move forward
  int sensorState =lineFinder.readSensors();
  bool onBlackMarker = (sensorState == S1_IN_S2_IN);
  if (onBlackMarker && !markerHandled){
    stop_motors();
    delay(100);

    int detectedColour =detectColour();
    instruction(detectedColour);
    markerHandled = true;
  } else if (!onBlackMarker){
    markerHandled = false;
  }
  double leftdistance = readLeftDistance();
  int irValue = readIR();

  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.print(" cm | IR: ");
  Serial.println(irValue);

  // 3. Stop if too close to a wall, otherwise drive
  if (leftDist < TOO_CLOSE_LEFT_CM){}
    nudgeRight();
  } else if (irValue >IR_TOO_CLOSE){
    nudgeLeft();
  }else {
  moveForward();
  }  
}





