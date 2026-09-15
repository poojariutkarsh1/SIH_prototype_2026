/*
  NEO-6M GPS + ESP8266 (NodeMCU) - Accurate Location Reader
  Library required: TinyGPSPlus (Sketch > Include Library > Manage Libraries > search "TinyGPSPlus")
*/

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Pin definitions
static const int RXPin = D4;   // ESP RX  <- GPS TX
static const int TXPin = D3;   // ESP TX  -> GPS RX
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin); // (RX, TX)

unsigned long lastPrint = 0;
const unsigned long printInterval = 1000; // print once per second

void setup() {
  Serial.begin(115200);      // Debug monitor
  gpsSerial.begin(GPSBaud);  // GPS module

  Serial.println();
  Serial.println("NEO-6M GPS Test - waiting for satellite fix...");
  Serial.println("(Keep the module outdoors / near a window with sky view)");
}

void loop() {
  // Feed the parser continuously - this is critical for accuracy,
  // TinyGPS++ needs every NMEA byte to build a valid sentence
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (millis() - lastPrint > printInterval) {
    lastPrint = millis();
    printGPSInfo();
  }

  // Warn if no data is coming from the module at all (wiring issue)
  if (millis() > 10000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS data received: check wiring (TX/RX swapped?) and baud rate.");
  }
}

void printGPSInfo() {
  if (gps.location.isValid()) {
    Serial.print("Latitude  : ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude : ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude  : ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.print("HDOP      : ");
    Serial.println(gps.hdop.hdop()); // lower = better accuracy; <2 is good
    Serial.print("Speed     : ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");
  } else {
    Serial.println("Location: Not fixed yet...");
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.printf("Date/Time (UTC): %02d/%02d/%04d %02d:%02d:%02d\n",
                  gps.date.day(), gps.date.month(), gps.date.year(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  Serial.println("-----------------------------");
}
