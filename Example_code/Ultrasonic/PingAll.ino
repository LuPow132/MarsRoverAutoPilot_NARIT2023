

/*
  Ultrasonic Sensor HC-SR04 and Arduino Tutorial

  by Dejan Nedelkovski,
  www.HowToMechatronics.com

*/
// defines pins numbers
const int trigPin_ut_1 = 22;
const int echoPin_ut_1 = 23;
int distance_1;

const int trigPin_ut_2 = 24;
const int echoPin_ut_2 = 25;
int distance_2;

const int trigPin_ut_3 = 26;
const int echoPin_ut_3 = 27;
int distance_3;

// defines variables
long duration;
int distance;

int ping(int trigPin,int echoPin){
  duration = 0;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance;
}

void pingAll(){
  distance_1 = ping(trigPin_ut_1,echoPin_ut_1);
  delay(10);
  distance_2 = ping(trigPin_ut_2,echoPin_ut_2);
  delay(10);
  distance_3 = ping(trigPin_ut_3,echoPin_ut_3);
  delay(10);
}

void setup() {
  pinMode(trigPin_ut_1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_ut_1, INPUT); // Sets the echoPin as an Input

  pinMode(trigPin_ut_2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_ut_2, INPUT); // Sets the echoPin as an Input

  pinMode(trigPin_ut_3, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_ut_3, INPUT); // Sets the echoPin as an Input

  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  pingAll();
  Serial.print("D1_");
  Serial.println(distance_1);

  Serial.print("D2_");
  Serial.println(distance_2);

  Serial.print("D3_");
  Serial.println(distance_3);
  
  Serial.println("======================");
  delay(100);
}
