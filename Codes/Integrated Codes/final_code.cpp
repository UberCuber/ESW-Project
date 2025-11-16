/*
 * ESP32 + ACD10 + AHT10 + MiCS-2714 + SDS011 (manual) + SD Card + ThingSpeak MQTT
 * Logs CO2, Temp, Humidity, NOx, PM2.5, PM10 locally and sends to ThingSpeak every 16s.
 * Includes non-blocking WiFi auto-reconnect logic.
 */

#include <WiFi.h>
#include <Wire.h>
#include "ACD10.h"
#include <Adafruit_AHTX0.h>
#include <SPI.h>
#include <SD.h>
#include <PubSubClient.h>
#include <time.h>

// ======== WiFi Settings ========
#define WIFI_SSID "myssid"
#define WIFI_PASSWORD "password"

// ======== ThingSpeak MQTT Settings ========
#define MQTT_SERVER "mqtt3.thingspeak.com"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "EDoSAwoaEDMtHSwlIykCOiw"      // Get from ThingSpeak account
#define MQTT_USERNAME "EDoSAwoaEDMtHSwlIykCOiw"        // Get from ThingSpeak account
#define MQTT_PASSWORD "hmV/sIsjeSLAyo39ntr6G56j"    // Get from ThingSpeak account
#define CHANNEL_ID "3124782"                      // Your ThingSpeak Channel ID
#define CHANNEL_WRITE_API_KEY "A5AC3GADNK7JIH88"

// ======== NTP Time Settings ========
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800; // India Standard Time (IST) is UTC+5:30
const int   daylightOffset_sec = 0;

// ======== MQTT Client ========
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ======== Sensors ========
ACD10 acd10;
Adafruit_AHTX0 aht;
#define MICS_AOUT_PIN 34
#define RL 10.0
#define VCC 5.0
float R0 = 10.0;

// ======== SDS011 (manual parsing) ========
#define SDS_RX 16
#define SDS_TX 17
HardwareSerial SDS_SERIAL(2);
float pm25 = -1.0, pm10 = -1.0;

// ======== SD Card ========
#define SD_CS 5
File logFile;

// ======== Timing ========
unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 16000UL;  // 16 seconds

// ======== WiFi auto-reconnect ========
unsigned long lastWifiAttempt = 0;
const unsigned long WIFI_RETRY_INTERVAL = 30000UL; // Try reconnecting every 30 seconds
bool wifiPreviouslyConnected = false;

// ======== Data Variables ========
float co2_ppm = -1.0;
float temp_C = -1.0, hum_percent = -1.0;
float nox_ppm = -1.0;

// ======== Prototypes ========
void connectToWiFi();
void initTime();
void initSD();
void logToSD(const String &line);
void connectMQTT();
void sendToThingSpeak();
bool readSDS011(float &out_pm25, float &out_pm10, unsigned long timeout = 1200);

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 Air Quality Monitor -> ThingSpeak MQTT + SD ===");

  // I2C & sensors
  Wire.begin(21, 22);
  Wire.setClock(100000);

  if (!acd10.begin()) {
    Serial.println("❌ ACD10 not found!");
    while (1) delay(1000);
  }
  Serial.println("✅ ACD10 initialized.");

  if (!aht.begin()) {
    Serial.println("❌ AHT10 not found!");
  } else {
    Serial.println("✅ AHT10 initialized.");
  }

  analogReadResolution(12);
  Serial.println("✅ MiCS-2714 analog ready on GPIO" + String(MICS_AOUT_PIN) + ".");

  // SDS011 serial (manual parsing)
  SDS_SERIAL.begin(9600, SERIAL_8N1, SDS_RX, SDS_TX);
  Serial.println("✅ SDS011 UART started (RX=" + String(SDS_RX) + ", TX=" + String(SDS_TX) + ").");

  // SD Card
  initSD();

  // WiFi & Time
  connectToWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    initTime();
    wifiPreviouslyConnected = true; // Set initial state
  }

    // MQTT Setup
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  Serial.println("✅ MQTT client configured.");

  // ======== ACD10 Warm-up ========
  Serial.println("\n🌡️  ACD10 CO₂ sensor warming up (3 minutes)...");
  unsigned long warmupStart = millis();
  const unsigned long warmupDuration = 3UL * 60UL * 1000UL; // 3 minutes in ms

  while (millis() - warmupStart < warmupDuration) {
    unsigned long remaining = (warmupDuration - (millis() - warmupStart)) / 1000;
    if (remaining % 10 == 0) { // Print every 10 seconds
      Serial.printf("⏳ Warming up... %lus remaining\n", remaining);
      delay(1000); // To avoid spamming
    }
    delay(900);
  }

  Serial.println("✅ ACD10 warm-up complete! Sensor ready for accurate readings.\n");

  Serial.println("============================================\n");
}

// ================== LOOP ==================
void loop() {
  unsigned long currentMillis = millis();

  // --- 1. WiFi & MQTT Connection Management (Runs every loop) ---
  if (WiFi.status() == WL_CONNECTED) {
    // We are connected to WiFi
    
    if (!wifiPreviouslyConnected) {
      // We *just* reconnected!
      Serial.println("\n✅ WiFi Re-connected! IP: " + WiFi.localIP().toString());
      initTime(); // Re-sync NTP time
      // MQTT will be handled below
    }
    wifiPreviouslyConnected = true;

    // Now that WiFi is up, manage MQTT
    if (!mqttClient.connected()) {
      Serial.println("MQTT disconnected, reconnecting...");
      connectMQTT(); // Try to connect MQTT (this has its own retries)
    }
    mqttClient.loop(); // Maintain MQTT connection

  } else {
    // We are disconnected from WiFi
    
    if (wifiPreviouslyConnected) {
      Serial.println("\n⚠️ WiFi connection lost. Will attempt to reconnect...");
    }
    wifiPreviouslyConnected = false;

    // Try to reconnect periodically (non-blocking)
    if (currentMillis - lastWifiAttempt >= WIFI_RETRY_INTERVAL) {
      lastWifiAttempt = currentMillis;
      Serial.print("Attempting WiFi reconnection...");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      // We just start the attempt, we don't wait (block) here.
    }
  }


  // --- 2. Sensor Reading Logic (Original code, runs on its interval) ---
  if (currentMillis - lastRead >= READ_INTERVAL) {
    lastRead = currentMillis;
    Serial.println("📊 --- Reading Sensors ---");

    // ACD10 CO2
    int reqStatus = acd10.requestSensor();
    if (reqStatus == 0) {
      while (!acd10.requestReady()) delay(10);
      if (acd10.readSensor() == 0) {
        co2_ppm = acd10.getCO2Concentration();
      } else {
        Serial.println("⚠️ ACD10 read error.");
      }
    }

    // AHT10 Temp & Humidity
    sensors_event_t humidity, temperature;
    aht.getEvent(&humidity, &temperature);
    temp_C = temperature.temperature;
    hum_percent = humidity.relative_humidity;

    // MiCS-2714 NOx (approx)
    int adc = analogRead(MICS_AOUT_PIN);
    float Vout = adc * (VCC / 4095.0);
    if (Vout <= 0.0) {
      nox_ppm = -1.0;
    } else {
      float Rs = RL * (VCC - Vout) / Vout;
      nox_ppm = pow(Rs / (4.0 * R0), 1.0 / 0.7) / 10.0;
    }

    // SDS011 manual read
    if (readSDS011(pm25, pm10)) {
      Serial.printf("✅ SDS011: PM2.5=%.1f µg/m³ | PM10=%.1f µg/m³\n", pm25, pm10);
    } else {
      Serial.println("⚠️ SDS011 read failed or timeout.");
    }

    // Print all
    Serial.printf("CO2: %.1f ppm | Temp: %.2f °C | Hum: %.1f %% | NOx: %.4f ppm | PM2.5: %.1f | PM10: %.1f\n",
                  co2_ppm, temp_C, hum_percent, nox_ppm, pm25, pm10);

    // Log to SD
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char timeStr[25];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);

      String logLine = String(timeStr) + "," +
                      String(co2_ppm, 2) + "," +
                      String(temp_C, 2) + "," +
                      String(hum_percent, 2) + "," +
                      String(nox_ppm, 4) + "," +
                      String(pm25, 1) + "," +
                      String(pm10, 1);
      logToSD(logLine);
    } else {
      Serial.println("⚠️ Failed to get time for logging.");
    }

    // Send to ThingSpeak
    // This function is safe, as it already checks for WiFi and MQTT connection
    sendToThingSpeak();

    Serial.println("------------------------------\n");
  }
}

// ================== WiFi & Time ==================
void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected! IP: " + WiFi.localIP().toString());

    // --- Add this block ---
    Serial.print("📶 Signal Strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    Serial.print("💻 MAC Address: ");
    Serial.println(WiFi.macAddress());

    Serial.print("🌐 IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("-----------------------------\n");
    // ------------------------
    
  } else {
    Serial.println("\n❌ WiFi Failed to connect.");
  }
}

void initTime() {
  Serial.println("Initializing time from NTP...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  const int maxAttempts = 5;
  int attempt = 0;

  while (attempt < maxAttempts) {
    if (getLocalTime(&timeinfo)) {
      Serial.println("✅ Time synchronized.");
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
      return;
    }

    attempt++;
    Serial.printf("⚠️ Failed to obtain time (attempt %d/%d). Retrying...\n", attempt, maxAttempts);
    delay(2000);
  }

  Serial.println("❌ Failed to obtain time after multiple attempts. Check Wi-Fi or NTP server.");
}

// ================== SD Card ==================
void initSD() {
  Serial.println("Initializing SD card...");
  SPI.begin(18, 19, 23, SD_CS);
  if (!SD.begin(SD_CS, SPI, 1000000)) {
    Serial.println("❌ SD Card Mount Failed!");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("❌ No SD card detected!");
    return;
  }

  Serial.print("✅ SD Card Type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC/SDXC");
  else Serial.println("Unknown");

  // Create file with header if doesn't exist
  if (!SD.exists("/sensor_log.csv")) {
    File f = SD.open("/sensor_log.csv", FILE_WRITE);
    if (f) {
      f.println("millis,co2_ppm,temp_C,hum_percent,nox_ppm,pm25,pm10");
      f.close();
      Serial.println("✅ Created sensor_log.csv with header.");
    }
  }

  Serial.println("SD Card initialized.\n");
}

void logToSD(const String &line) {
  logFile = SD.open("/sensor_log.csv", FILE_APPEND);
  if (logFile) {
    logFile.println(line);
    logFile.close();
    Serial.println("💾 Logged to SD: " + line);
  } else {
    Serial.println("⚠️ Failed to open file for logging!");
  }
}

// ================== MQTT Connection ==================
void connectMQTT() {
  Serial.print("Connecting to ThingSpeak MQTT...");
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 3) {
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("\n✅ MQTT Connected!");
      return;
    } else {
      Serial.print("❌ MQTT failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying...");
      delay(2000);
      attempts++;
    }
  }
}

// ================== ThingSpeak MQTT Upload ==================
void sendToThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi not connected, skipping ThingSpeak upload.");
    return;
  }

  if (!mqttClient.connected()) {
    Serial.println("⚠️ MQTT not connected, skipping ThingSpeak upload.");
    return;
  }

  // ThingSpeak MQTT topic format
  String topic = "channels/" + String(CHANNEL_ID) + "/publish";
  
  // Build payload in ThingSpeak format
  // field1=CO2, field2=Temp, field3=Humidity, field4=NOx, field5=PM2.5, field6=PM10
  String payload = "field1=" + String(co2_ppm, 2) +
                   "&field2=" + String(temp_C, 2) +
                   "&field3=" + String(hum_percent, 2) +
                   "&field4=" + String(nox_ppm, 4) +
                   "&field5=" + String(pm25, 1) +
                   "&field6=" + String(pm10, 1) +
                   "&status=MQTTPUBLISH";

  // Publish to ThingSpeak
  if (mqttClient.publish(topic.c_str(), payload.c_str())) {
    Serial.println("📤 ThingSpeak MQTT upload successful!");
    Serial.println("Payload: " + payload);
  } else {
    Serial.println("❌ ThingSpeak MQTT upload failed!");
  }
}

// ================== SDS011 Manual Read ==================
bool readSDS011(float &out_pm25, float &out_pm10, unsigned long timeout) {
  uint8_t buf[10];
  unsigned long start = millis();

  while (millis() - start < timeout) {
    if (SDS_SERIAL.available() >= 1) {
      int b = SDS_SERIAL.read();
      if (b == 0xAA) {
        unsigned long waitStart = millis();
        while (SDS_SERIAL.available() < 9 && millis() - waitStart < 200) {
          delay(2);
        }
        if (SDS_SERIAL.available() >= 9) {
          buf[0] = 0xAA;
          for (int i = 1; i < 10; i++) buf[i] = (uint8_t)SDS_SERIAL.read();
          
          if (buf[1] == 0xC0 && buf[9] == 0xAB) {
            uint8_t checksum = 0;
            for (int i = 2; i <= 7; i++) checksum += buf[i];
            
            if (checksum == buf[8]) {
              uint16_t raw_pm25 = (buf[3] << 8) | buf[2];
              uint16_t raw_pm10 = (buf[5] << 8) | buf[4];
              out_pm25 = raw_pm25 / 10.0;
              out_pm10 = raw_pm10 / 10.0;
              return true;
            }
          }
        }
      }
    }
    delay(2);
  }

  return false;
}