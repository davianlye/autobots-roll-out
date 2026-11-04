// COLOUR SENSOR TEST - AVERAGED (no motors, no PID)
// Takes a batch of SAMPLES readings while you slide the robot around on one paper,
// then prints each sample, the average, min/max/spread per channel, the detected colour,
// and a ready-to-paste RAW_REF line.
// Serial Monitor: 115200 baud.

#include <MeMCore.h>

MeRGBLed led(7, 2);
MeBuzzer buzzer;

#define DecoderA A3
#define DecoderB A2
#define ldrPin A1
#define ledSettleTime 100

#define SAMPLES 10          // readings per batch (each takes ~0.3 s -> ~3 s per batch)
#define PAUSE_BETWEEN 3000  // ms between batches: time to move to the next paper

// ---- Your latest calibration (same as the main sketch) ----
const int RAW_REF[6][3] = {
  {551, 866, 514},  // 1 Red
  {738, 748, 610},  // 2 Green
  {657, 883, 527},  // 3 Orange
  {758, 887, 697},  // 4 Pink
  {724, 720, 705},  // 5 Light blue
  {806, 876, 742}  // 6 White
};
//377, 525, 338 black
const float redOrangeSplit = 0.56;   // same red/orange tie-breaker as the main sketch

const char* NAMES[7] = {"UNKNOWN", "RED", "GREEN", "ORANGE", "PINK", "LIGHT BLUE", "WHITE"};

// ---- Decoder ----
void shineIR()    { digitalWrite(DecoderA, LOW);  digitalWrite(DecoderB, LOW);  } // colour LEDs off
void shineRed()   { digitalWrite(DecoderA, HIGH); digitalWrite(DecoderB, LOW);  }
void shineGreen() { digitalWrite(DecoderA, LOW);  digitalWrite(DecoderB, HIGH); }
void shineBlue()  { digitalWrite(DecoderA, HIGH); digitalWrite(DecoderB, HIGH); }

long colourReading() {
  delay(ledSettleTime);
  long total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(ldrPin);
    delay(5);
  }
  return total / 5;
}

// One RGB sample (same order as the main sketch)
void readRGB(int out[3]) {
  shineRed();   out[0] = colourReading();
  shineGreen(); out[1] = colourReading();
  shineBlue();  out[2] = colourReading();
  shineIR();
}

// Same matching as the main sketch: nearest raw reference + red/orange ratio tie-breaker.
int classify(float rgb[3], float &bestDist, int &second, float &secondDist) {
  int best = 1;
  bestDist = 100000; secondDist = 100000; second = 1;
  for (int c = 0; c < 6; c++) {
    float dr = rgb[0] - RAW_REF[c][0];
    float dg = rgb[1] - RAW_REF[c][1];
    float db = rgb[2] - RAW_REF[c][2];
    float dist = sqrt(dr * dr + dg * dg + db * db);
    if (dist < bestDist) {
      second = best; secondDist = bestDist;
      best = c + 1;  bestDist = dist;
    } else if (dist < secondDist) {
      second = c + 1; secondDist = dist;
    }
  }
  if (best == 1 || best == 3) {
    best = (rgb[0] / rgb[1] < redOrangeSplit) ? 1 : 3;
  }
  return best;
}

void showColour(int colour) {
  if (colour == 1)      led.setColor(0, 255, 0, 0);
  else if (colour == 2) led.setColor(0, 0, 255, 0);
  else if (colour == 3) led.setColor(0, 255, 100, 0);
  else if (colour == 4) led.setColor(0, 255, 60, 150);
  else if (colour == 5) led.setColor(0, 100, 200, 255);
  else if (colour == 6) led.setColor(0, 255, 255, 255);
  else                  led.setColor(0, 0, 0, 0);
  led.show();
}

void setup() {
  Serial.begin(115200);
  pinMode(DecoderA, OUTPUT);
  pinMode(DecoderB, OUTPUT);
  shineIR();
  showColour(0);
  Serial.println("Averaged colour test ready.");
}

void loop() {
  // ---- Countdown so you can place the robot ----
  showColour(0);
  Serial.println();
  Serial.println("=== Place robot on the paper. Starting in 3 s... ===");
  delay(PAUSE_BETWEEN);
  buzzer.tone(1000, 100);   // beep = start moving the robot around the paper
  Serial.println("SAMPLING - slide the robot around the paper now");

  long sum[3] = {0, 0, 0};
  int mn[3] = {1023, 1023, 1023};
  int mx[3] = {0, 0, 0};
  int votes[7] = {0, 0, 0, 0, 0, 0, 0};

  for (int s = 0; s < SAMPLES; s++) {
    int r[3];
    readRGB(r);

    float rf[3] = {(float)r[0], (float)r[1], (float)r[2]};
    float d, d2; int sec;
    int c = classify(rf, d, sec, d2);
    votes[c]++;

    Serial.print("  #"); Serial.print(s + 1);
    Serial.print("  {"); Serial.print(r[0]); Serial.print(", ");
    Serial.print(r[1]); Serial.print(", "); Serial.print(r[2]);
    Serial.print("}  -> "); Serial.println(NAMES[c]);

    for (int k = 0; k < 3; k++) {
      sum[k] += r[k];
      if (r[k] < mn[k]) mn[k] = r[k];
      if (r[k] > mx[k]) mx[k] = r[k];
    }
  }
  buzzer.tone(1500, 100);   // beep = done

  // ---- Summary ----
  float avg[3];
  for (int k = 0; k < 3; k++) avg[k] = (float)sum[k] / SAMPLES;

  float bestDist, secondDist; int second;
  int result = classify(avg, bestDist, second, secondDist);
  showColour(result);

  int agree = votes[result];

  Serial.println("--------------- SUMMARY ---------------");
  Serial.print("AVERAGE  {"); Serial.print(avg[0], 0); Serial.print(", ");
  Serial.print(avg[1], 0); Serial.print(", "); Serial.print(avg[2], 0); Serial.println("}");
  Serial.print("MIN      {"); Serial.print(mn[0]); Serial.print(", ");
  Serial.print(mn[1]); Serial.print(", "); Serial.print(mn[2]); Serial.println("}");
  Serial.print("MAX      {"); Serial.print(mx[0]); Serial.print(", ");
  Serial.print(mx[1]); Serial.print(", "); Serial.print(mx[2]); Serial.println("}");
  Serial.print("SPREAD   {"); Serial.print(mx[0] - mn[0]); Serial.print(", ");
  Serial.print(mx[1] - mn[1]); Serial.print(", "); Serial.print(mx[2] - mn[2]);
  Serial.println("}   (max - min; under ~30 is good)");
  Serial.print("R/O RATIO "); Serial.println(avg[0] / avg[1], 3);

  Serial.print(">>> DETECTED (from average): "); Serial.print(NAMES[result]);
  Serial.print("   dist "); Serial.print(bestDist, 0);
  Serial.print(", runner-up "); Serial.print(NAMES[second]);
  Serial.print(" at "); Serial.println(secondDist, 0);
  Serial.print(">>> Individual samples agreeing: ");
  Serial.print(agree); Serial.print("/"); Serial.println(SAMPLES);

  Serial.print("PASTE INTO RAW_REF:  {");
  Serial.print(avg[0], 0); Serial.print(", ");
  Serial.print(avg[1], 0); Serial.print(", ");
  Serial.print(avg[2], 0); Serial.println("},");
  Serial.println("---------------------------------------");
}
