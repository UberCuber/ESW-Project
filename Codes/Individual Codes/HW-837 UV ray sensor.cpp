// --- HW-837 UV Sensor with ESP32 ---
// Analog pin for HW-837 UV sensor
const int uvPin = 34;  // Use any ADC-capable pin on ESP32

void setup() {
  Serial.begin(115200);
  Serial.println("HW-837 UV Sensor Test");
}

void loop() {
  // Read the raw analog value (ESP32 ADC range: 0–4095)
  int uvValue = analogRead(uvPin);

  // Convert the analog value to voltage (ESP32 ADC is 3.3V max)
  float uvVoltage = uvValue / 4095.0 * 3.3;

  // Rough conversion: UV Index ~ Voltage / 0.1
  // (you should calibrate this for your sensor if precision matters)
  float uvIndex = uvVoltage / 0.1;

  // Print readings
  Serial.print("UV Analog Value: ");
  Serial.println(uvValue);

  Serial.print("UV Voltage: ");
  Serial.print(uvVoltage);
  Serial.println(" V");

  Serial.print("UV Index: ");
  Serial.println(uvIndex);

  Serial.println(); // blank line
  delay(1000);
}