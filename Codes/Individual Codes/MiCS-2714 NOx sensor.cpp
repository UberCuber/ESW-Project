#define MICS_AOUT_PIN 34
#define RL 10.0 // kOhms
#define VCC 5

float R0 = 10.0; // baseline resistance in kOhms (calibrate first)

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop() {
  int adc = analogRead(MICS_AOUT_PIN);
  float Vout = adc * (VCC / 4095.0);

  float Rs = RL * (VCC - Vout) / Vout;

  float concentration = pow(Rs / (4.0 * R0), 1.0 / 0.7)/10;

  Serial.print("Raw ADC: "); Serial.print(adc);
  Serial.print(" | Rs: "); Serial.print(Rs);
  Serial.print(" kOhms | Concentration: "); Serial.print(concentration);
  Serial.println(" ppm");

  delay(1000);
}