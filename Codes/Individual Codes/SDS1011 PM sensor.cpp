#include <HardwareSerial.h>

HardwareSerial sds(1);  // use UART1

#define RXD2 16   // RX pin (ESP32 side) → TX pin of SDS011
#define TXD2 17   // TX pin (ESP32 side) → RX pin of SDS011

void setup() {
  Serial.begin(115200); 
  sds.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println("SDS011 sensor test started...");
}

void loop() {
  static uint8_t buf[10];
  static int idx = 0;

  while (sds.available()) {
    uint8_t b = sds.read();

    // Look for frame start
    if (idx == 0 && b != 0xAA) continue; 

    buf[idx++] = b;

    if (idx == 10) {
      idx = 0;  // reset buffer

      // Validate packet header/footer
      if (buf[0] == 0xAA && buf[1] == 0xC0 && buf[9] == 0xAB) {
        uint16_t pm25 = (buf[3] << 8) | buf[2];  // µg/m³ * 10
        uint16_t pm10 = (buf[5] << 8) | buf[4];  

        float pm25_value = pm25 / 10.0;
        float pm10_value = pm10 / 10.0;

        Serial.print("PM2.5: ");
        Serial.print(pm25_value);
        Serial.print(" µg/m³, PM10: ");
        Serial.print(pm10_value);
        Serial.println(" µg/m³");
      }
    }
  }
}