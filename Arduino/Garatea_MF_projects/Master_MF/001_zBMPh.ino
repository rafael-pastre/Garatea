#ifndef ZBMP_H
#define ZBMP_H

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

class zBMP{
  public:
    zBMP();                       // object constructor
    boolean init(uint8_t mode = BMP085_ULTRAHIGHRES); // initialization of bmp sensor
    void read(void);                  // read all data from sensor
    void readRawTemperature(void);            // read temperature from sensor
    void readRawPressure(void);             // read pressure from sensor
    
    uint16_t raw_temperature(void);           // getter for temperature
    uint32_t raw_pressure(void);            // getter for pressure
    
    int16_t get_ac1(void);                // getter for ac1
    int16_t get_ac2(void);                // getter for ac2
    int16_t get_ac3(void);                // getter for ac3
    uint16_t get_ac4(void);               // getter for ac4
    uint16_t get_ac5(void);               // getter for ac5
    uint16_t get_ac6(void);               // getter for ac6
    
    int16_t get_b1(void);               // getter for b1
    int16_t get_b2(void);               // getter for b2
    int16_t get_mb(void);               // getter for mb
    int16_t get_mc(void);               // getter for mc
    int16_t get_md(void);               // getter for md
    
  private:
    uint8_t oversampling;

    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    
    uint32_t BMP_pressure;                      // pressure
    uint16_t BMP_temperature;                   // temperature

    int32_t computeB5(int32_t UT);
    uint8_t read8(uint8_t addr);
    uint16_t read16(uint8_t addr);
    void write8(uint8_t addr, uint8_t data);
};

#endif//ZBMP_H
