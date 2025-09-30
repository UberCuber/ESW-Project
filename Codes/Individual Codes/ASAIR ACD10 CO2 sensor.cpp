#include <HardwareSerial.h>

HardwareSerial acd10(1);  // use UART1

#define RX_PIN 16   // ESP32 RX → ACD10 TX
#define TX_PIN 17   // ESP32 TX → ACD10 RX

void setup() {
  Serial.begin(115200);
  acd10.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("ACD10 CO2 sensor test started...");
}

void loop() {
  if (acd10.available() >= 9) {
    byte response[9];
    acd10.readBytes(response, 9);

    if (response[0] == 0xFF && response[1] == 0x86) {
      int high = response[2];
      int low  = response[3];
      int co2ppm = (high << 8) | low;

      Serial.print("CO₂ Concentration: ");
      Serial.print(co2ppm);
      Serial.println(" ppm");
    }
  }
}