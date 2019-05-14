#ifndef ZBMP_H
#define ZBMP_H

#include "zstdlib.h"
#include <Adafruit_BMP085.h>                      // BMP Library

// BMP GLOBAL DATA VARIABLES
extern Adafruit_BMP085 bmp180;                    // defines the BMP sensor
extern int32_t BMP_pressure;                      // pressure
extern int32_t BMP_temperature;                   // temperature
extern int32_t BMP_Altitude;                      // altitude

// BMP USER FUNCTIONS
err_t BMP_init();
err_t BMP_read();
err_t BMP_read_Pressure(int32_t* pressure);
err_t BMP_read_Altitude(int32_t* alt);
err_t BMP_read_Temperature(int32_t* temperature);

// DEBUG
#ifdef DEBUG_MODE
void BMP_print_Pressure();
void BMP_print_Altitude();
void BMP_print_Temperature();
void BMP_print_Info();
#endif//DEBUG_MODE

#endif//ZBMP_H