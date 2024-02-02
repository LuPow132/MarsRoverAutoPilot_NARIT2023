#include <SPI.h>  
#include "RF24.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 11, TXPin = 10;
static const uint32_t GPSBaud = 9600;

RF24 myRadio (7, 8);
byte addresses[][6] = {"0"};
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

struct package {
  int id=1;
  double rover_lat = 0.0;
  double rover_long = 0.0;
  int sat_used = 0;
};


typedef struct package Package;
Package data;


void setup() {
  Serial.begin(115200);
  delay(1000);
  ss.begin(GPSBaud);
  
  myRadio.begin();  
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS );
  myRadio.openWritingPipe(addresses[0]);
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      Serial.print("\nPackage:");
      Serial.print(data.id);
      Serial.print("\n");
      Serial.println(data.rover_lat);
      Serial.println(data.rover_long);
      Serial.println(data.sat_used);
      data.id = data.id + 1;
      data.rover_lat = gps.location.lat();
      data.rover_long = gps.location.lng();
      data.sat_used = gps.satellites.value();
      myRadio.write(&data, sizeof(data)); 
    }
  }
    
}
