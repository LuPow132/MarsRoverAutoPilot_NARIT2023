#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 11, TXPin = 10;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
}

void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      if(gps.satellites.value() > 9){
        // Latitude in degrees (double)
        Serial.print(gps.location.lat(), 6);
        // Longitude in degrees (double)
        Serial.print(",");
        Serial.println(gps.location.lng(), 6);
      }

      // Serial.print("Number os satellites in use = ");
      // Serial.println(gps.satellites.value());
    }
  }
}
