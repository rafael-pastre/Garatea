#include <zenith.h>

zBMP bmp;
zBMPdata bmp_data;

void setup() {
  #ifdef DEBUG_MODE
    Serial.begin(DEBUG_BAUD);
    Serial.println("Debug Mode");
  #endif
  
  while (!bmp.init()){
    #ifdef DEBUG_MODE
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    #endif
  }
  
  bmp_data.set_ac1(bmp.get_ac1());
  bmp_data.set_ac2(bmp.get_ac2());
  bmp_data.set_ac3(bmp.get_ac3());
  bmp_data.set_ac4(bmp.get_ac4());
  bmp_data.set_ac5(bmp.get_ac5());
  bmp_data.set_ac6(bmp.get_ac6());
  bmp_data.set_b1(bmp.get_b1());
  bmp_data.set_b2(bmp.get_b2());
  bmp_data.set_mb(bmp.get_mb());
  bmp_data.set_mc(bmp.get_mc());
  bmp_data.set_md(bmp.get_md());
}

void loop() {
  // put your main code here, to run repeatedly:
  bmp.read();
  #ifdef DEBUG_MODE
    Serial.print("Raw pressure: ");Serial.print(bmp_data.raw_pressure);Serial.print(", Raw temp: ");Serial.println(bmp_data.raw_temperature);
  #endif
  bmp_data.raw_pressure = bmp.raw_pressure();
  bmp_data.raw_temperature = bmp.raw_temperature();

  #ifdef DEBUG_MODE
    Serial.print("Converted pressure: ");Serial.print(bmp_data.convertPressure());Serial.print(" Converted temp: ");Serial.println(bmp_data.convertTemperature());
    Serial.print("Sea Level Pressure: ");Serial.print(bmp_data.convertSealevelPressure());Serial.print(" Altitude: ");Serial.println(bmp_data.convertAltitude());Serial.println();
  #endif
  
  delay(5000);
}
