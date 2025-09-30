#include <Adafruit_AHTX0.h>

// Create the AHT sensor object
Adafruit_AHTX0 aht;

// Define the pin for the UV sensor
const int uvPin = 34; // Analog input pin for the HW-837 sensor

void setup() {
  // Start serial communication
  Serial.begin(115200);
  Serial.println("Combined AHT10/20 and UV Sensor Test");

  // Initialize the AHT10/AHT20 sensor
  if (!aht.begin()) {
    Serial.println("Could not find AHT sensor. Check wiring!");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found.");
}

void loop() {
  // --- Read Temperature and Humidity Sensor (AHT10) ---
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); // Populate temp and humidity objects with fresh data

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %rH");

  // --- Read UV Sensor (HW-837) ---
  // Read the raw analog value (0-4095 on ESP32)
  int uvValue = analogRead(uvPin); 
  
  // Convert the analog value to voltage (ESP32 ADC is 3.3V)
  float uvVoltage = uvValue / 4095.0 * 3.3;

  // The UV index is roughly proportional to the output voltage.
  // This is an approximation; you may need to calibrate it for precise measurements.
  float uvIndex = uvVoltage / 0.1; // A common conversion factor for these sensors

  Serial.print("UV Analog Value: ");
  Serial.println(uvValue);
  Serial.print("UV Index: ");
  Serial.println(uvIndex);

  // Print a blank line for readability and wait
  Serial.println("");
  delay(1000); 
}