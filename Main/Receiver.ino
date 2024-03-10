#include <SPI.h>  
#include "RF24.h" 
#include <TinyGPS++.h>
RF24 myRadio (7, 8); 
byte addresses[][6] = {"0"};

struct package {
  int id=1;
  double rover_lat = 0.0;
  double rover_long = 0.0;
  int sat_used = 0;
  int distance_1 = 0;
  int distance_2 = 0;
  int distance_3 = 0;
  int distance_4 = 0;
  int distance_5 = 0;
  int distance_6 = 0;
  int distance_7 = 0;
};

typedef struct package Package;
Package data;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting up...");
  delay(1000);

  myRadio.begin(); 
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS );
  Serial.println("Found NRF module");
  
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();
}


void loop()  {
  if ( myRadio.available()) {
    while (myRadio.available()){
      myRadio.read( &data, sizeof(data) );
    }
    Serial.print("\nPackage:");
    Serial.print("Package/");
    Serial.println(data.id);
    Serial.print("lat/");
    Serial.println(data.rover_lat);
    Serial.print("long/");
    Serial.println(data.rover_long);
    Serial.println("Distance/");
    Serial.println(data.distance_1);
    Serial.println(data.distance_2);
    Serial.println(data.distance_3);
    Serial.println(data.distance_4);
    Serial.println(data.distance_5);
    Serial.println(data.distance_6);
    Serial.println(data.distance_7);

  }
}
