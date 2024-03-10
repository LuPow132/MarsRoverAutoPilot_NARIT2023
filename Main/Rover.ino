#include <SPI.h>  
#include "RF24.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

const int trigPin_ut_1 = 22;
const int echoPin_ut_1 = 23;
int distance_1;

const int trigPin_ut_2 = 24;
const int echoPin_ut_2 = 25;
int distance_2;

const int trigPin_ut_3 = 26;
const int echoPin_ut_3 = 27;
int distance_3;

const int trigPin_ut_4 = 28;
const int echoPin_ut_4 = 29;
int distance_4;

const int trigPin_ut_5 = 30;
const int echoPin_ut_5 = 31;
int distance_5;

const int trigPin_ut_6 = 32;
const int echoPin_ut_6 = 33;
int distance_6;

const int trigPin_ut_7 = 34;
const int echoPin_ut_7 = 35;
int distance_7;

long duration;
int distance;
int trig,echo;

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

int ping(int trig,int echo){
  digitalWrite(trig, LOW); 
  delayMicroseconds(5); 
  digitalWrite(trig, HIGH); 
  delayMicroseconds(5); 
  digitalWrite(trig, LOW); //ใช้งานขา trig
  
  duration = pulseIn(echo, HIGH); //อ่านค่าของ echo
  distance = (duration/2) / 29.1; //คำนวณเป็น centimeters
  return distance;
}

void setup() {
  Serial.begin(115200);
  Serial.println("booting UP");
  delay(1000);
  ss.begin(GPSBaud);
  Serial.println("found GPS module..");
  
  myRadio.begin();  
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS );
  myRadio.openWritingPipe(addresses[0]);
  Serial.println("found NRF module..");

  pinMode(echoPin_ut_1, INPUT); //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(trigPin_ut_1, OUTPUT); //สั่งให้ขา trig ใช้งานเป็น output
  pinMode(echoPin_ut_2, INPUT); //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(trigPin_ut_2, OUTPUT); //สั่งให้ขา trig ใช้งานเป็น output
  pinMode(echoPin_ut_3, INPUT); //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(trigPin_ut_3, OUTPUT); //สั่งให้ขา trig ใช้งานเป็น output
  pinMode(echoPin_ut_4, INPUT); //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(trigPin_ut_4, OUTPUT); //สั่งให้ขา trig ใช้งานเป็น output
  pinMode(echoPin_ut_5, INPUT); //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(trigPin_ut_5, OUTPUT); //สั่งให้ขา trig ใช้งานเป็น output
  pinMode(echoPin_ut_6, INPUT); //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(trigPin_ut_6, OUTPUT); //สั่งให้ขา trig ใช้งานเป็น output
  pinMode(echoPin_ut_7, INPUT); //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(trigPin_ut_7, OUTPUT); //สั่งให้ขา trig ใช้งานเป็น output
}

void loop() {
  while (ss.available() > 0) {
    data.id = data.id + 1;
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      data.rover_lat = gps.location.lat();
      data.rover_long = gps.location.lng();
      data.sat_used = gps.satellites.value();
    }else{
      data.rover_lat = 0.0;
      data.rover_long = 0.0;
      data.sat_used = 0;
    }

    data.distance_1 = ping(trigPin_ut_1,echoPin_ut_1);
    data.distance_2 = ping(trigPin_ut_2,echoPin_ut_2);
    data.distance_3 = ping(trigPin_ut_3,echoPin_ut_3);
    data.distance_4 = ping(trigPin_ut_4,echoPin_ut_4);
    data.distance_5 = ping(trigPin_ut_5,echoPin_ut_5);
    data.distance_6 = ping(trigPin_ut_6,echoPin_ut_6);
    data.distance_7 = ping(trigPin_ut_7,echoPin_ut_7);
    
    myRadio.write(&data, sizeof(data)); 
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

    delay(300);
  }
}
