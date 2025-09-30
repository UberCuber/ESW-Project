#include <HardwareSerial.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include <time.h>

// ======== SDS011 ========
HardwareSerial sds(1);
#define SDS_RX 16
#define SDS_TX 17

// ======== MiCS-2714 ========
#define MICS_AOUT_PIN 34
#define RL 10.0
#define VCC 5.0
float R0 = 10.0;

// ======== SGP30 ========
Adafruit_SGP30 sgp;

// ======== Timing ========
time_t startTimeUnix;
unsigned long startMillis;
unsigned long prevLoopMillis = 0;
const long loopInterval = 5000; // 5 seconds

// ======== Data Variables ========
float globalPM25 = -1;
float globalPM10 = -1;
float nox_ppm = -1;
int eCO2 = -1, TVOC = -1;
char timeBuffer[30];

// ======== Function Prototypes ========
bool checkSDS();

String getTimeString() {
  unsigned long elapsedSeconds = (millis() - startMillis) / 1000;
  time_t currentTimeUnix = startTimeUnix + elapsedSeconds;
  struct tm* timeinfo = localtime(&currentTimeUnix);
  strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  return String(timeBuffer);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Merged SDS011 + MiCS-2714 + SGP30 with Software Ticking Clock");

  // ===== Time Initialization =====
  setenv("TZ", "IST-5:30", 1); // IST timezone
  tzset();
  const char* compileTimeStr = __DATE__ " " __TIME__;
  struct tm tm;
  if (strptime(compileTimeStr, "%b %d %Y %H:%M:%S", &tm) != NULL) {
    startTimeUnix = mktime(&tm);
    startMillis = millis();
    Serial.print("Software clock start time set to: ");
    Serial.println(compileTimeStr);
  } else {
    Serial.println("ERROR: Failed to parse compile time!");
  }

  // ===== SDS011 =====
  sds.begin(9600, SERIAL_8N1, SDS_RX, SDS_TX);

  // ===== MiCS-2714 =====
  analogReadResolution(12);

  // ===== SGP30 =====
  Wire.begin();
  if (!sgp.begin()) {
    Serial.println("SGP30 not found!");
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - prevLoopMillis >= loopInterval) {
    prevLoopMillis = currentMillis;

    // --- SDS011 ---
    checkSDS();

    // --- MiCS-2714 ---
    int adc = analogRead(MICS_AOUT_PIN);
    float Vout = adc * (VCC / 4095.0);
    nox_ppm = (Vout == 0) ? 0 : pow(RL * (VCC / Vout - 1.0) / (4.0 * R0), 1.0 / 0.7);

    // --- SGP30 ---
    if (sgp.IAQmeasure()) {
      eCO2 = sgp.eCO2;
      TVOC = sgp.TVOC;
    }

    // --- PRINT READINGS ---
    Serial.println("-------------------------------------------------");
    Serial.print("Timestamp: "); Serial.println(getTimeString());
    Serial.println("Sensor Readings:");
    Serial.print("  PM2.5: "); Serial.print(globalPM25); Serial.println(" µg/m³");
    Serial.print("  PM10 : "); Serial.print(globalPM10); Serial.println(" µg/m³");
    Serial.print("  NOx  : "); Serial.print(nox_ppm); Serial.println(" ppm");
    Serial.print("  eCO2 : "); Serial.print(eCO2); Serial.println(" ppm");
    Serial.print("  TVOC : "); Serial.print(TVOC); Serial.println(" ppb");
    Serial.println("-------------------------------------------------\n");
  }
}

bool checkSDS() {
  byte buffer[10];
  while (sds.available() > 0) {
    if (sds.peek() != 0xAA) {
      sds.read();
      continue;
    }
    if (sds.available() < 10) {
      return false;
    }
    sds.readBytes(buffer, 10);
    if (buffer[0] == 0xAA && buffer[1] == 0xC0 && buffer[9] == 0xAB) {
      globalPM25 = ((buffer[3] * 256) + buffer[2]) / 10.0;
      globalPM10 = ((buffer[5] * 256) + buffer[4]) / 10.0;
      return true;
    }
  }
  return false;
}