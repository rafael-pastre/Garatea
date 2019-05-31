#include "zGPS.h"

zGPS gps1(10, 11, 12, 9600, 150);

void setup() {
  #ifdef DEBUG_MODE
    Serial.begin(DEBUG_BAUD);
    Serial.println("Debug Mode");
  #endif
  
  gps1.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  gps1.read();
  Serial.print("Lat: ");Serial.println(gps1.lat());
  Serial.print("Lon: ");Serial.println(gps1.lon());
  
  delay(200);
}
