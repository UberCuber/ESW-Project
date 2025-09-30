#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include <time.h>

// ======== WiFi Credentials ========
const char* ssid = "Mahek'ssss";
const char* password = "jhanu@2008";

// ======== ThingSpeak MQTT Settings ========
const char* mqttServer = "mqtt3.thingspeak.com";
const int mqttPort = 1883;
const char* mqttClientID = "CCoGMTAHHAYiPRMcPRcMKjE";        // Any unique ID
const char* mqttUsername = "CCoGMTAHHAYiPRMcPRcMKjE";    // From ThingSpeak Account
const char* mqttPassword = "6lMEepGfQibXrxb0/V5Ve3bp";    // From ThingSpeak Account
const char* writeAPIKey = "CR21Q8HXPR6477JY";     // From your ThingSpeak Channel
const long channelID = 3093335;             // Your ThingSpeak Channel ID

// ======== MQTT Client ========
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

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
unsigned long lastMQTTPublish = 0;
const long loopInterval = 5000;      // 5 seconds for readings
const long publishInterval = 15000;   // 15 seconds for ThingSpeak (free account limit)

// ======== Data Variables ========
float globalPM25 = -1;
float globalPM10 = -1;
float nox_ppm = -1;
int eCO2 = -1, TVOC = -1;
char timeBuffer[30];

// ======== Function Prototypes ========
bool checkSDS();
void connectToWiFi();
void connectToMQTT();
void publishToThingSpeak();
String getTimeString();

String getTimeString() {
  unsigned long elapsedSeconds = (millis() - startMillis) / 1000;
  time_t currentTimeUnix = startTimeUnix + elapsedSeconds;
  struct tm* timeinfo = localtime(&currentTimeUnix);
  strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  return String(timeBuffer);
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("✅ WiFi Connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("❌ WiFi Connection Failed!");
  }
}

void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to ThingSpeak MQTT...");
    
    if (mqttClient.connect(mqttClientID, mqttUsername, mqttPassword)) {
      Serial.println(" ✅ Connected to ThingSpeak!");
    } else {
      Serial.print(" ❌ Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void publishToThingSpeak() {
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  
  // Create the topic string
  String topicString = "channels/" + String(channelID) + "/publish";
  
  // Create the payload with field data
  String payload = "field1=" + String(globalPM25) +
                  "&field2=" + String(globalPM10) +
                  "&field3=" + String(nox_ppm) +
                  "&field4=" + String(eCO2) +
                  "&status=Published from ESP32 at " + getTimeString();
  
  Serial.println("----- Publishing to ThingSpeak -----");
  Serial.println("Topic: " + topicString);
  Serial.println("Payload: " + payload);
  
  if (mqttClient.publish(topicString.c_str(), payload.c_str())) {
    Serial.println("✅ Data published successfully!");
    Serial.println("🌐 Check your ThingSpeak channel: https://thingspeak.com/channels/" + String(channelID));
  } else {
    Serial.println("❌ Failed to publish data");
  }
  Serial.println("------------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=======================================");
  Serial.println("ESP32 Air Quality Monitor -> ThingSpeak");
  Serial.println("=======================================");

  // ===== WiFi Connection =====
  connectToWiFi();
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Cannot proceed without WiFi. Please check credentials.");
    while(1) delay(1000);
  }

  // ===== MQTT Setup =====
  mqttClient.setServer(mqttServer, mqttPort);
  connectToMQTT();

  // ===== Time Initialization =====
  setenv("TZ", "IST-5:30", 1); // IST timezone
  tzset();
  const char* compileTimeStr = __DATE__ " " __TIME__;
  struct tm tm;
  if (strptime(compileTimeStr, "%b %d %Y %H:%M:%S", &tm) != NULL) {
    startTimeUnix = mktime(&tm);
    startMillis = millis();
    Serial.println("⏰ Software clock initialized: " + String(compileTimeStr));
  } else {
    Serial.println("❌ Failed to initialize time");
  }

  // ===== SDS011 =====
  sds.begin(9600, SERIAL_8N1, SDS_RX, SDS_TX);
  Serial.println("🌬️  SDS011 PM sensor initialized");

  // ===== MiCS-2714 =====
  analogReadResolution(12);
  Serial.println("🏭 MiCS-2714 NOx sensor initialized");

  // ===== SGP30 =====
  Wire.begin();
  if (sgp.begin()) {
    Serial.println("🌿 SGP30 CO2/TVOC sensor initialized");
  } else {
    Serial.println("❌ SGP30 not found!");
  }

  Serial.println("=======================================");
  Serial.println("🚀 System Ready! Starting measurements...");
  Serial.println("📊 Readings every " + String(loopInterval/1000) + "s");
  Serial.println("📤 ThingSpeak publish every " + String(publishInterval/1000) + "s");
  Serial.println("=======================================\n");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop();
  
  // Take sensor readings
  if (currentMillis - prevLoopMillis >= loopInterval) {
    prevLoopMillis = currentMillis;

    // --- SDS011 PM Sensor ---
    bool sdsSuccess = checkSDS();

    // --- MiCS-2714 NOx Sensor ---
    int adc = analogRead(MICS_AOUT_PIN);
    float Vout = adc * (VCC / 4095.0);
    nox_ppm = (Vout == 0) ? 0 : pow(RL * (VCC / Vout - 1.0) / (4.0 * R0), 1.0 / 0.7);

    // --- SGP30 CO2/TVOC Sensor ---
    bool sgpSuccess = false;
    if (sgp.IAQmeasure()) {
      eCO2 = sgp.eCO2;
      TVOC = sgp.TVOC;
      sgpSuccess = true;
    }

    // --- Display Current Readings ---
    Serial.println("📊 CURRENT READINGS 📊");
    Serial.println("Time: " + getTimeString());
    Serial.println("PM2.5: " + String(globalPM25) + " µg/m³ " + (sdsSuccess ? "✅" : "❌"));
    Serial.println("PM10:  " + String(globalPM10) + " µg/m³ " + (sdsSuccess ? "✅" : "❌"));
    Serial.println("NOx:   " + String(nox_ppm) + " ppm ✅");
    Serial.println("eCO2:  " + String(eCO2) + " ppm " + (sgpSuccess ? "✅" : "❌"));
    Serial.println("TVOC:  " + String(TVOC) + " ppb " + (sgpSuccess ? "✅" : "❌"));
    Serial.println("------------------------");
  }

  // Publish to ThingSpeak
  if (currentMillis - lastMQTTPublish >= publishInterval) {
    lastMQTTPublish = currentMillis;
    publishToThingSpeak();
  }
}

bool checkSDS() {
  byte buffer[10];
  bool dataReceived = false;
  
  while (sds.available() > 0) {
    if (sds.peek() != 0xAA) {
      sds.read();
      continue;
    }
    if (sds.available() < 10) {
      break;
    }
    sds.readBytes(buffer, 10);
    if (buffer[0] == 0xAA && buffer[1] == 0xC0 && buffer[9] == 0xAB) {
      globalPM25 = ((buffer[3] * 256) + buffer[2]) / 10.0;
      globalPM10 = ((buffer[5] * 256) + buffer[4]) / 10.0;
      dataReceived = true;
      break;
    }
  }
  return dataReceived;
}