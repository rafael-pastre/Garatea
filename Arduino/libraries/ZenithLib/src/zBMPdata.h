#ifndef ZBMPDATA_H
#define ZBMPDATA_H

#include "zstdlib.h"

#define BMP085_I2CADDR 0x77

#define BMP085_ULTRALOWPOWER     0
#define BMP085_STANDARD          1
#define BMP085_HIGHRES           2
#define BMP085_ULTRAHIGHRES      3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4 
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD       0x2E
#define BMP085_READPRESSURECMD   0x34

class zBMPdata{
	public:
		zBMPdata(uint8_t mode = BMP085_ULTRAHIGHRES);
		
		uint32_t raw_pressure;
		uint16_t raw_temperature;
		
		int32_t convertPressure(void);
		int32_t convertSealevelPressure(float altitude_meters = 0);
		float convertTemperature(void);
		float convertAltitude(float sealevelPressure = 101325);
		
		void set_ac1(int16_t x);
		void set_ac2(int16_t x);
		void set_ac3(int16_t x);
		void set_ac4(uint16_t x);
		void set_ac5(uint16_t x);
		void set_ac6(uint16_t x);
		
		void set_b1(int16_t x);
		void set_b2(int16_t x);
		
		void set_mb(int16_t x);
		void set_mc(int16_t x);
		void set_md(int16_t x);
		
	private:
		int32_t computeB5(int32_t UT);
		
		uint8_t oversampling;
		int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
		uint16_t ac4, ac5, ac6;
};
#endif//ZBMPDATA_H
