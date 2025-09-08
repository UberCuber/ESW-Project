#include "MQUnifiedsensor.h"

#define Board "ESP-32"
#define Pin_MQ135 34
#define Voltage_Resolution 3.3
#define Type "MQ-135" // Sensor type
#define ADC_Bit_Resolution 12 // For ESP32 ADC
#define RatioMQ135CleanAir 3.6 // RS / R0 ratio in fresh air

// Define the sensor
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin_MQ135, Type);

void setup() {
  Serial.begin(115200);

  // Set the calibration method (using PPM formula)
  MQ135.setRegressionMethod(1); //_PPM = a*ratio^b
  
  // Set A and B coefficients for MQ-135
  // Found in the sensor datasheet or through online resources
  MQ135.setA(100);
  MQ135.setB(-2.6);

  Serial.print("Calibrating the sensor, please wait.");
  float calcR0 = 0;
  for(int i = 1; i <= 10; i++) {
    MQ135.update(); // Update data from the sensor
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }

  // Set the calibrated R0 value
  MQ135.setR0(calcR0/10);
  Serial.println(" done!");

  if(isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite. Check wiring!");
    while(1);
  }
  if(calcR0 == 0) {
    Serial.println("Warning: Connection issue, R0 is zero. Check wiring!");
    while(1);
  }
  
  Serial.println("R0 value: ");
  Serial.println(MQ135.getR0());
}

void loop() {
  MQ135.update(); // Update sensor data
  
  // Read sensor resistance and PPM values
  float ppm = MQ135.readSensor();
  
  // Print the values to the Serial Monitor
  Serial.print("PPM: ");
  Serial.println(ppm);
  
  delay(1000); // Wait for a second before the next reading
}



// connections:
// MQ-135 Pin    ESP32 Pin    Description
// VCC    5V    Power supply for the sensor.
// GND    GND    Ground.
// AO    GPIO 34    Analog output, connected to an ESP32 analog input pin.
// DO    (Optional)    Digital output, can be connected to any GPIO pin.
