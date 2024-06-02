#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Kalman filter variables
double kalmanLat, kalmanLng;
double P_lat = 1, P_lng = 1; // Estimate error
double R = 0.1; // Measurement noise
double Q = 0.0001; // Process noise

static const int RXPin = 17, TXPin = 16;
static const uint32_t GPSBaud = 115200;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void kalmanFilter(double newLat, double newLng)
{
  // Latitude
  double K_lat = P_lat / (P_lat + R);
  kalmanLat = kalmanLat + K_lat * (newLat - kalmanLat);
  P_lat = (1 - K_lat) * P_lat + fabs(kalmanLat - newLat) * Q;

  // Longitude
  double K_lng = P_lng / (P_lng + R);
  kalmanLng = kalmanLng + K_lng * (newLng - kalmanLng);
  P_lng = (1 - K_lng) * P_lng + fabs(kalmanLng - newLng) * Q;
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    double rawLat = gps.location.lat();
    double rawLng = gps.location.lng();

    // Apply the Kalman filter
    kalmanFilter(rawLat, rawLng);

    Serial.print(kalmanLat, 6);
    Serial.print(F(","));
    Serial.print(kalmanLng, 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
