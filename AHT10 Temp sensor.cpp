#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit AHT10/AHT20 Test!");

  if (!aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degrees C");

  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println("% rH");

  Serial.println("");
  delay(1000); // Wait 5 seconds before the next reading
}



// temperature sensor connections:
// AHT10 Pin    ESP32 Pin    Description
// VCC    3.3V    Power supply for the sensor.
// GND    GND    Ground.
// SDA    GPIO 21    I²C Serial Data Line.
// SCL    GPIO 22    I²C Serial Clock Line.
