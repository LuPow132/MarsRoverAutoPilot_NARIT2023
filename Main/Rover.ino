#include <SPI.h>  
#include "RF24.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <IBusBM.h>
#include <QMC5883LCompass.h>

//Left side Motor
int dir1PinL = 5;

int dir2PinL = 6;

int speedPinL = 7;


//Right side Motor

int dir1PinR = 3;

int dir2PinR = 4;

int speedPinR = 2;

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

unsigned long prevTimeSendData = millis();
int intervalTimeSendData = 300;

unsigned long prevTimePingAll = millis();
int intervalTimePingAll = 1000;

RF24 myRadio (8, 9);
byte addresses[][6] = {"0"};
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
IBusBM ibus;
QMC5883LCompass compass;

int compass_heading = 0;
float distance_from_target = 0;
int direction_to_target = 0;
int mode = 0; 
int RX,RY;
int compass_value;
bool reach_destination = false;
bool gps_value_reach = false;
int heading_threshold = 8;
int satellites_amount_to_start = 4;
/*
mode variable use number to represent mode that it currently are rightnow based on this

0 = manual drive(flysky remote)
1 = autopilot
*/

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
  char message = "";
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

// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void autopilot(double destination_lat,double destination_long){
  reach_destination = false;
  gps_value_reach = false;
  while(!reach_destination){
    while (ss.available() > 0){
      if (gps.encode(ss.read())){
        if(gps_value_reach){
          if (gps.location.isUpdated()){
            distance_from_target = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), destination_lat, destination_long);
            direction_to_target = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_long);

            //Destination Location
            Serial.print(destination_lat,6);
            Serial.print(",");
            Serial.println(destination_long,6);

            //Current Location
            Serial.print(gps.location.lat(),6);
            Serial.print(",");
            Serial.println(gps.location.lng(),6);

            Serial.print("Distance from destination is: ");
            Serial.println(distance_from_target);

            Serial.print("Direction to destination is: ");
            Serial.println(direction_to_target);

            compass.read();
            compass_value = compass.getAzimuth();
            if(compass_value < 0){
              compass_value = 360 + compass_value;
            }

            Serial.print("Compass value : ");
            Serial.println(compass_value);

            int rot_error = findRotError(compass_value,direction_to_target);

            if(abs(rot_error) < heading_threshold){
              Serial.println("Heading Forward");
            }else{
              Serial.print("Rotate error: ");
              Serial.println(rot_error);
            }
          }
        }else{
          if (gps.satellites.value() < satellites_amount_to_start){
            Serial.println(gps.satellites.value());
          }else{
            gps_value_reach = true;
          }
        }
      }
    }
  }
}

void send_data(){
  data.id = data.id + 1;
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
}

void motor_drive(int motor_left_speed,int motor_right_speed){

  int mapped_speed_L = map(abs(motor_left_speed),0,100,0,255);
  if(motor_left_speed > 0){
    analogWrite(speedPinL, mapped_speed_L);

    digitalWrite(dir1PinL, LOW);

    digitalWrite(dir2PinL, HIGH);

  }else if(motor_left_speed < 0){
    analogWrite(speedPinL, mapped_speed_L);

    digitalWrite(dir1PinL, HIGH);

    digitalWrite(dir2PinL, LOW);
  }else if(motor_left_speed == 0){
    analogWrite(speedPinL, 0);

    digitalWrite(dir1PinL, LOW);

    digitalWrite(dir2PinL, LOW);
  }

  int mapped_speed_R = map(abs(motor_right_speed),0,100,0,255);
  if(motor_right_speed > 0){
    analogWrite(speedPinR, mapped_speed_R);

    digitalWrite(dir1PinR, LOW);

    digitalWrite(dir2PinR, HIGH);

  }else if(motor_right_speed < 0){
    analogWrite(speedPinR, mapped_speed_R);

    digitalWrite(dir1PinR, HIGH);

    digitalWrite(dir2PinR, LOW);
  }else if(motor_right_speed == 0){
    analogWrite(speedPinR, 0);

    digitalWrite(dir1PinR, LOW);

    digitalWrite(dir2PinL, LOW);
  }
}

int findRotError(int compass_value, int direction_need_to_head_value) {
    // Calculate the absolute clockwise and counterclockwise differences
    int clockwise_diff = (direction_need_to_head_value - compass_value + 360) % 360;
    int counterclockwise_diff = (compass_value - direction_need_to_head_value + 360) % 360;

    // Determine the minimum difference and direction
    int rotation = (clockwise_diff <= counterclockwise_diff) ? clockwise_diff : -counterclockwise_diff;

    // Ensure the rotation is within the range of -180 to 180
    if (rotation > 180)
        rotation -= 360;
    else if (rotation <= -180)
        rotation += 360;

    return rotation;
}

void waypoint_execute(double Latitude_array[],double Longtitude_array[]){
  //get amount of each array
  int Lat_num_Elements = sizeof(Latitude_array) / sizeof(Latitude_array[0]);
  int Long_num_Elements = sizeof(Longtitude_array) / sizeof(Longtitude_array[0]);

  //Check if 2 array has the same amount of element
  if(Lat_num_Elements != Long_num_Elements){
    Serial.println("array amount not matching!");
    return false;
  }

  for(int i = 0; i < Lat_num_Elements; i++){
    autopilot(Latitude_array[i],Longtitude_array[i]);
  }
  Serial.println("Successfuly execute autopilot for all waypoint!");
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("boot up");
  delay(300);
  ss.begin(GPSBaud);
  Serial.println("Connected to GPS");
  
  myRadio.begin();  
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS );
  myRadio.openWritingPipe(addresses[0]);
  Serial.println("Connected to NRF");

  data.id = data.id + 1;
  data.message = "Radio Connected!";
  myRadio.write(&data, sizeof(data)); 

  ibus.begin(Serial1);
  Serial.println("Connected to IBUS");

  compass.init();
  Serial.println("Connected to compass");
  compass.setCalibrationOffsets(587.00, -530.00, 99.00);
  compass.setCalibrationScales(0.81, 1.13, 1.13);

  pinMode(dir1PinL,OUTPUT);
  pinMode(dir2PinL,OUTPUT);
  pinMode(speedPinL,OUTPUT);
  pinMode(dir1PinR,OUTPUT);
  pinMode(dir2PinR,OUTPUT);
  pinMode(speedPinR,OUTPUT);

  pinMode(echoPin_ut_1, INPUT); 
  pinMode(trigPin_ut_1, OUTPUT); 
  pinMode(echoPin_ut_2, INPUT); 
  pinMode(trigPin_ut_2, OUTPUT);
  pinMode(echoPin_ut_3, INPUT); 
  pinMode(trigPin_ut_3, OUTPUT); 
  pinMode(echoPin_ut_4, INPUT); 
  pinMode(trigPin_ut_4, OUTPUT); 
  pinMode(echoPin_ut_5, INPUT); 
  pinMode(trigPin_ut_5, OUTPUT); 
  pinMode(echoPin_ut_6, INPUT); 
  pinMode(trigPin_ut_6, OUTPUT); 
  pinMode(echoPin_ut_7, INPUT); 
  pinMode(trigPin_ut_7, OUTPUT); 

  
  // autopilot(13.276906029602102, 100.92121938129964);
}

void loop() {
  //manual
  while(mode == 0){
    unsigned long currentime = millis();

    RX = readChannel(0, -100, 100, 0);
    RY = -1 * readChannel(1, -100, 100, 0);

    gps.encode(ss.read());
    data.rover_lat = gps.location.lat();
    data.rover_long = gps.location.lng();
    data.sat_used = gps.satellites.value();
    int R = RX + RY;
    int L = RX - RY;
    Serial.print(L);
    Serial.print("|");
    Serial.println(R);
    motor_drive(L,R);

    if(currentime - prevTimeSendData > intervalTimeSendData){
      send_data();
      prevTimeSendData = currentime;
    }

  }
  //auto
  while(mode == 1){
    unsigned long currentime = millis();

    gps.encode(ss.read());
    data.rover_lat = gps.location.lat();
    data.rover_long = gps.location.lng();
    data.sat_used = gps.satellites.value();

    if(currentime - prevTimePingAll > intervalTimePingAll){
      data.distance_1 = ping(trigPin_ut_1,echoPin_ut_1);
      data.distance_2 = ping(trigPin_ut_2,echoPin_ut_2);
      data.distance_3 = ping(trigPin_ut_3,echoPin_ut_3);
      data.distance_4 = ping(trigPin_ut_4,echoPin_ut_4);
      data.distance_5 = ping(trigPin_ut_5,echoPin_ut_5);
      data.distance_6 = ping(trigPin_ut_6,echoPin_ut_6);
      data.distance_7 = ping(trigPin_ut_7,echoPin_ut_7);

      prevTimePingAll = currentime;
    }

    //Send data every 500ms using threading
    if(currentime - prevTimeSendData > intervalTimeSendData){
      send_data();
      prevTimeSendData = currentime;
    }
  }
}
