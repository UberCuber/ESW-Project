#include <ThreeWire.h>
#include <RtcDS1302.h>

// Define DS1302 pins (adjust for ESP32)
ThreeWire myWire(19, 18, 21);  // DAT, CLK, RST (choose suitable ESP32 GPIOs)
RtcDS1302<ThreeWire> Rtc(myWire);

void setup() {
  Serial.begin(115200);

  Rtc.Begin();

  // Optional: Set time initially (uncomment only once)
// RtcDateTime correctTime(2025, 9, 15, 10, 42, 0); // (year, month, day, hour, min, sec)
//   Rtc.SetDateTime(correctTime);

// }

void loop() {
  RtcDateTime now = Rtc.GetDateTime();

  Serial.print("Date: ");
  Serial.print(now.Day());
  Serial.print("/");
  Serial.print(now.Month());
  Serial.print("/");
  Serial.println(now.Year());

  Serial.print("Time: ");
  Serial.print(now.Hour());
  Serial.print(":");
  Serial.print(now.Minute());
  Serial.print(":");
  Serial.println(now.Second());

  delay(1000);
}

void setManualTime(int year, int month, int day, int hour, int minute, int second) {
  RtcDateTime newTime(year, month, day, hour, minute, second);
  Rtc.SetDateTime(newTime);
}