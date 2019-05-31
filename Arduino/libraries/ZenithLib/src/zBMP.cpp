#include "zBMP.h"

zBMP::zBMP() {
	pressure = 0;
	temperature = 0;
}

boolean zBMP::init(uint8_t mode) {
  if (mode > BMP085_ULTRAHIGHRES) 
    mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;

  Wire.begin();

  if (read8(0xD0) != 0x55) return false;

  /* read calibration data */
  ac1 = read16(BMP085_CAL_AC1);
  ac2 = read16(BMP085_CAL_AC2);
  ac3 = read16(BMP085_CAL_AC3);
  ac4 = read16(BMP085_CAL_AC4);
  ac5 = read16(BMP085_CAL_AC5);
  ac6 = read16(BMP085_CAL_AC6);

  b1 = read16(BMP085_CAL_B1);
  b2 = read16(BMP085_CAL_B2);

  mb = read16(BMP085_CAL_MB);
  mc = read16(BMP085_CAL_MC);
  md = read16(BMP085_CAL_MD);
#ifdef DEBUG_BMP
  Serial.print(F("ac1 = ")); Serial.println(ac1, DEC);
  Serial.print(F("ac2 = ")); Serial.println(ac2, DEC);
  Serial.print(F("ac3 = ")); Serial.println(ac3, DEC);
  Serial.print(F("ac4 = ")); Serial.println(ac4, DEC);
  Serial.print(F("ac5 = ")); Serial.println(ac5, DEC);
  Serial.print(F("ac6 = ")); Serial.println(ac6, DEC);

  Serial.print(F("b1 = ")); Serial.println(b1, DEC);
  Serial.print(F("b2 = ")); Serial.println(b2, DEC);

  Serial.print(F("mb = ")); Serial.println(mb, DEC);
  Serial.print(F("mc = ")); Serial.println(mc, DEC);
  Serial.print(F("md = ")); Serial.println(md, DEC);
#endif

  return true;
}

void zBMP::read(void){
	zBMP::readRawTemperature();
	zBMP::readRawPressure();
	#ifdef DEBUG_BMP
		Serial.println(F(" "));
	#endif
}

void zBMP::readRawTemperature(void) {
	uint16_t temp;
	write8(BMP085_CONTROL, BMP085_READTEMPCMD);
	delay(5);
	
	temp = read16(BMP085_TEMPDATA);
	if(temp && temp != 0x00FF)
		temperature = temp;
#ifdef DEBUG_BMP
	Serial.print(F("Raw temp: ")); Serial.println(temperature);
#endif
}

void zBMP::readRawPressure(void) {
	uint32_t pres;
  write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP085_ULTRALOWPOWER) 
    delay(5);
  else if (oversampling == BMP085_STANDARD) 
    delay(8);
  else if (oversampling == BMP085_HIGHRES) 
    delay(14);
  else 
    delay(26);

  pres = read16(BMP085_PRESSUREDATA);

  pres <<= 8;
  pres |= read8(BMP085_PRESSUREDATA+2);
  pres >>= (8 - oversampling);

	if(pres && pres != 0x00FFFFFF)
		pressure = pres;
#ifdef DEBUG_BMP
  Serial.print(F("Raw pressure: ")); Serial.println(pressure);
#endif

}

/********************************************************************/

int32_t zBMP::computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

uint8_t zBMP::read8(uint8_t a) {
  uint8_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
#else
  Wire.send(a); // sends register address to read from
#endif
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
#if (ARDUINO >= 100)
  ret = Wire.read(); // receive DATA
#else
  ret = Wire.receive(); // receive DATA
#endif
  Wire.endTransmission(); // end transmission

  return ret;
}

uint16_t zBMP::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
#else
  Wire.send(a); // sends register address to read from
#endif
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
#if (ARDUINO >= 100)
  ret = Wire.read(); // receive DATA
  ret <<= 8;
  ret |= Wire.read(); // receive DATA
#else
  ret = Wire.receive(); // receive DATA
  ret <<= 8;
  ret |= Wire.receive(); // receive DATA
#endif
  Wire.endTransmission(); // end transmission

  return ret;
}

void zBMP::write8(uint8_t a, uint8_t d) {
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
  Wire.write(d);  // write data
#else
  Wire.send(a); // sends register address to read from
  Wire.send(d);  // write data
#endif
  Wire.endTransmission(); // end transmission
}