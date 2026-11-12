// Serial Monitor: 115200 baud.

#include <MeMCore.h>

MeBuzzer buzzer;

#define DecoderA A3
#define DecoderB A2
#define ldrPin A1
#define ledSettleTime 100

#define SAMPLES 10          // readings per batch
#define PAUSE_BETWEEN 3000  // time delay to move to the next paper

// Decoder
void shineIR()    { digitalWrite(DecoderA, LOW);  digitalWrite(DecoderB, LOW);  } // colour LEDs off
void shineRed()   { digitalWrite(DecoderA, HIGH); digitalWrite(DecoderB, LOW);  }
void shineGreen() { digitalWrite(DecoderA, LOW);  digitalWrite(DecoderB, HIGH); }
void shineBlue()  { digitalWrite(DecoderA, HIGH); digitalWrite(DecoderB, HIGH); }

// Same reading method as the main code: settle, then average 5 analogRead values
long colourReading() {
  delay(ledSettleTime);
  long total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(ldrPin);
    delay(5);
  }
  return total / 5;
}

// One RGB sample (same order as the main code)
void readRGB(int out[3]) {
  shineRed();   out[0] = colourReading();
  shineGreen(); out[1] = colourReading();
  shineBlue();  out[2] = colourReading();
  shineIR();
}

void setup() {
  Serial.begin(115200);
  pinMode(DecoderA, OUTPUT);
  pinMode(DecoderB, OUTPUT);
  shineIR();
  Serial.println("Colour calibration ready.");
}

void loop() {
  // Countdown to place the robot
  Serial.println();
  Serial.println("Place robot on the paper. Starting in 3 s");
  delay(PAUSE_BETWEEN);
  buzzer.tone(1000, 100);   // beep = start moving the robot around the paper
  Serial.println("Slide the robot around the paper now");

  long sum[3] = {0, 0, 0};

  for (int s = 0; s < SAMPLES; s++) {
    int r[3];
    readRGB(r);

    Serial.print("  #"); Serial.print(s + 1);
    Serial.print("  {"); Serial.print(r[0]); Serial.print(", ");
    Serial.print(r[1]); Serial.print(", "); Serial.print(r[2]); Serial.println("}");

    for (int k = 0; k < 3; k++) sum[k] += r[k];
  }
  buzzer.tone(1500, 100);   // beep = done

  //Average of the 10 readings
  float avg[3];
  for (int k = 0; k < 3; k++) avg[k] = (float)sum[k] / SAMPLES;

  Serial.print("AVERAGE  {"); Serial.print(avg[0], 0); Serial.print(", ");
  Serial.print(avg[1], 0); Serial.print(", "); Serial.print(avg[2], 0); Serial.println("}");

  Serial.print("PASTE INTO RAW_REF:  {");
  Serial.print(avg[0], 0); Serial.print(", ");
  Serial.print(avg[1], 0); Serial.print(", ");
  Serial.print(avg[2], 0); Serial.println("},");
  Serial.println("---------------------------------------");
}
