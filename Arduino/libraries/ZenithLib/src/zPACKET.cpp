#include "zPACKET.h"

// Constructor
zPACKET::zPACKET(uint8_t mode) {
	BMP_pressure = 0;
	BMP_temperature = 0;
	
	if (mode > BMP085_ULTRAHIGHRES) 
		mode = BMP085_ULTRAHIGHRES;
	oversampling = mode;
}

// Data convertion
int32_t zPACKET::convertPressure(void) {
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = BMP_temperature;
  UP = BMP_pressure;

#ifdef DEBUG_BMP_CAL
  Serial.print(F("UT: "));Serial.println(UT);
  Serial.print(F("UP: "));Serial.println(UP);
  Serial.print(F("Oversampling: "));Serial.println(oversampling);
  Serial.print(F("a1: "));Serial.print(ac1);
  Serial.print(F(" a2: "));Serial.print(ac2);
  Serial.print(F(" a3: "));Serial.print(ac3);
  Serial.print(F(" a4: "));Serial.print(ac4);
  Serial.print(F(" a5: "));Serial.print(ac5);
  Serial.print(F(" a6: "));Serial.println(ac6);
  Serial.print(F("b1: "));Serial.print(b1);
  Serial.print(F(" b2: "));Serial.println(b2);
  Serial.print(F("mb: "));Serial.print(mb);
  Serial.print(F(" mc: "));Serial.print(mc);
  Serial.print(F(" md: "));Serial.println(md);
  
#endif
  B5 = computeB5(UT);

#ifdef DEBUG_BMP_CAL
  Serial.print(F("X1 = ")); Serial.println(X1);
  Serial.print(F("X2 = ")); Serial.println(X2);
  Serial.print(F("B5 = ")); Serial.println(B5);
#endif

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

#ifdef DEBUG_BMP_CAL
  Serial.print(F("B6 = ")); Serial.println(B6);
  Serial.print(F("X1 = ")); Serial.println(X1);
  Serial.print(F("X2 = ")); Serial.println(X2);
  Serial.print(F("B3 = ")); Serial.println(B3);
#endif

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

#ifdef DEBUG_BMP_CAL
  Serial.print(F("X1 = ")); Serial.println(X1);
  Serial.print(F("X2 = ")); Serial.println(X2);
  Serial.print(F("B4 = ")); Serial.println(B4);
  Serial.print(F("B7 = ")); Serial.println(B7);
#endif

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

#ifdef DEBUG_BMP_CAL
  Serial.print(F("p = ")); Serial.println(p);
  Serial.print(F("X1 = ")); Serial.println(X1);
  Serial.print(F("X2 = ")); Serial.println(X2);
#endif

  p = p + ((X1 + X2 + (int32_t)3791)>>4);
#ifdef DEBUG_BMP_CAL
  Serial.print(F("p = ")); Serial.println(p);
#endif
  return p;
}

int32_t zPACKET::convertSealevelPressure(float altitude_meters) {
  float pressure = convertPressure();
  return (int32_t)(pressure / pow(1.0-altitude_meters/44330, 5.255));
}

float zPACKET::convertTemperature(void) {
  int32_t UT, B5;     // following ds convention
  float temp;

  UT = BMP_temperature;

  B5 = computeB5(UT);
  temp = (B5+8) >> 4;
  temp /= 10;
  
  return temp;
}

float zPACKET::convertAltitude(float sealevelPressure) {
  float altitude;

  float pressure = convertPressure();

  altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

  return altitude;
}


// Auxiliar functions
int32_t zPACKET::computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}