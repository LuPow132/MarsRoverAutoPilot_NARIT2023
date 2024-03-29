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
  Serial.begin(9600);
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
  Serial.print(ping(trigPin_ut_1,echoPin_ut_1)); 
  Serial.print(" | ");
  Serial.print(ping(trigPin_ut_2,echoPin_ut_2)); 
  Serial.print(" | ");
  Serial.print(ping(trigPin_ut_3,echoPin_ut_3)); 
  Serial.print(" | ");
  Serial.print(ping(trigPin_ut_4,echoPin_ut_4)); 
  Serial.print(" | ");
  Serial.print(ping(trigPin_ut_5,echoPin_ut_5)); 
  Serial.print(" | ");
  Serial.print(ping(trigPin_ut_6,echoPin_ut_6)); 
  Serial.print(" | ");
  Serial.println(ping(trigPin_ut_7,echoPin_ut_7)); 
  delay(100);
}
