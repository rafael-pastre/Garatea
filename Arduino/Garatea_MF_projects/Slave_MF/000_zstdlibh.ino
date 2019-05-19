#ifndef ZSTDLIB_H
#define ZSTDLIB_H

#include <Wire.h>                                 // I2C Library
#include <SoftwareSerial.h>                       // UART Library
#include <stdlib.h>

// DEBUG OPTIONS
#define DEBUG_MODE
#define DEBUG_MODE_HIGH
#define DEBUG_BAUD 	115200
#define DEBUG_DELAY	5000

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
#define BMP_PRES_TSID   0                         // transmissionState value for sending BMP raw pression to master
#define BMP_TEMP_TSID   1                         // transmissionState value for sending BMP raw temperature to master
#define BMP_CAL_TSID    2                         // transmissionState value for sending BMP calibration data to master

#define GPS_TIME_TSID   3                         // transmissionState value for sending GPS time to master
#define GPS_LAT_TSID    4                         // transmissionState value for sending GPS latitude to master
#define GPS_NS_TSID     5                         // transmissionState value for sending GPS North/South to master
#define GPS_LONG_TSID   6                         // transmissionState value for sending GPS longitude to master
#define GPS_EW_TSID     7                         // transmissionState value for sending GPS East/West to master
#define GPS_QUAL_TSID   8                         // transmissionState value for sending GPS quality to master
#define GPS_SAT_TSID    9                         // transmissionState value for sending GPS satellites number to master
#define GPS_HDOP_TSID   10                        // transmissionState value for sending GPS horizontal precision to master
#define GPS_AGE_TSID    11                        // transmissionState value for sending GPS information age to master


//int32_t i2c_read_int32();
int32_t swap_bytes (int32_t v);
void swap_bytes (void* str, size_t sz);
int32_t findStrIndex(byte* str, int32_t st_index, int32_t end_index, char c);

#ifdef DEBUG_MODE
void printString(byte* str, int len);
#endif
#endif//ZSTDLIB_H
