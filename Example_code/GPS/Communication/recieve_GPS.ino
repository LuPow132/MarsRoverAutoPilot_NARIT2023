#include <SPI.h>  
#include "RF24.h" 
#include <TinyGPS++.h>
RF24 myRadio (7, 8); 
byte addresses[][6] = {"0"};

double landing_lat = 13.276444;
double landing_long =  100.921656;
struct package {
  int id=0;
  double rover_lat = 0.0;
  double rover_long = 0.0;
  int sat_used = 0;
}; 

typedef struct package Package;
Package data;

void setup() {
  Serial.begin(115200);
  delay(1000);

  myRadio.begin(); 
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS );
  
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();
}


void loop()  {
  if ( myRadio.available()) {
    while (myRadio.available()){
      myRadio.read( &data, sizeof(data) );
    }
    Serial.print("\nPackage:");
    Serial.print(data.id);
    Serial.print("\n");
    Serial.println(data.rover_lat, 6);
    Serial.println(data.rover_long, 6);
    Serial.println(data.sat_used);
    Serial.print("The distance from local to GPS is: ");
    Serial.println(TinyGPSPlus::distanceBetween(landing_lat, landing_long  , data.rover_lat,  data.rover_long));

  }
}
