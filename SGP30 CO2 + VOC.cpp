#include <Wire.h>
#include "Adafruit_SGP30.h"

Adafruit_SGP30 sgp;

void setup() {
  Serial.begin(115200);
  
  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.println("Sensor found!");
}

void loop() {
  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  
  Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
  Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");
  
  delay(1000); // Wait 1 second before the next reading
}

// connections:
// SGP30 Pin    ESP32 Pin    Description
// VCC    3.3V    Power supply for the sensor.
// GND    GND    Ground.
// SDA    GPIO 21    I²C Serial Data Line.
// SCL    GPIO 22    I²C Serial Clock Line.
