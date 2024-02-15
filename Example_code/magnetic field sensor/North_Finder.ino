
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  compass.init();
  compass.setSmoothing(10,true); 
  
}

void loop() {
  int x;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  
  if(abs(x) <= 45){
    Serial.println("North");
  }else{
    Serial.println("Not North");
  }
  
  delay(100);
}
