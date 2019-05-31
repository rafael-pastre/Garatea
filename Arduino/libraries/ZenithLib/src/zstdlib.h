#ifndef ZSTDLIB_H
#define ZSTDLIB_H

#if ARDUINO >= 100                                // Arduino Default Library
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>                                 // I2C Library
#include <SoftwareSerial.h>                       // UART Library
#include <stdlib.h>

// DEBUG OPTIONS
//#define DEBUG_MODE
#define DEBUG_MODE_HIGH
#define DEBUG_BAUD 	115200
#define DEBUG_DELAY	5000

//#ifdef DEBUG_MODE
//#define DEBUG_MODE_HIGH
//#endif
// ERROR HANDLING (may be deprecated with Object Orientation)
typedef uint8_t         err_t;                    // definition of type used for error handling
#define NO_ERR          0                         // sucessful execution
#define INDEX_ERR       1                         // index error
#define READ_ERR        2                         // read error
#define INV_DATA_ERR    3                         // invalid data
#define TRANSMISS_ERR   4                         // transmission error
#define INIT_ERR        5                         // initialization error

// TRASMISSION STATE IDs
// transmissionState defines what information the slave will send to master
// must set transmissionState before every read call on the slave
#define BMP_CAL_TSID    1						// transmissionState value for sending BMP calibration data to master
#define BMP_DATA_TSID   2						// transmissionState value for sending BMP sensor data to master
#define GPS1_DATA_TSID  3						// transmissionState value for sending GPS1 sensor data to master
#define GPS2_DATA_TSID  4						// transmissionState value for sending GPS2 sensor data to master

// Standard functions
int32_t swap_bytes (int32_t v);
void swap_bytes (void* str, size_t sz);
int32_t findStrIndex(byte* str, int32_t st_index, int32_t end_index, char c);
int32_t absolute(int32_t x);

#if defined DEBUG_MODE || defined DEBUG_MODE_HIGH
void printString(byte* str, int len);
#endif
#endif//ZSTDLIB_H