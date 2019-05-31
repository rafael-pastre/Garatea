#ifndef ZSTDLIB_H
#define ZSTDLIB_H

#include <Wire.h>									// I2C Library
#include <SoftwareSerial.h>							// UART Library
#include <SPI.h>									// SPI Library
#include <LoRa.h>									// LoRa Library
#include <SD.h>										// SD Library
#include <avr/wdt.h>								// WatchDogTimer Library
#include <stdlib.h>									// C Standard Library

// Debug constants
#define DEBUG_BAUD		115200
#define DEBUG_DELAY		5000

//#ifdef DEBUG_MODE
//#define DEBUG_MODE_HIGH
//#endif

/*
 *	Error handling (may be deprecated with Object Orientation)
 */
typedef uint8_t			err_t;						// definition of type used for error handling
#define NO_ERR			0							// sucessful execution
#define INDEX_ERR		1							// index error
#define READ_ERR		2							// read error
#define INV_DATA_ERR	3							// invalid data
#define TRANSMISS_ERR	4							// transmission error
#define INIT_ERR		5							// initialization error


/*
 *	Transmission State IDs
 *	transmissionState defines what information the slave will send to master
 *	must set transmissionState before every read call on the slave
 */
#define BMP_CAL_TSID	1							// transmissionState value for sending BMP calibration data to master
#define BMP_DATA_TSID	2							// transmissionState value for sending BMP raw data to master
#define GPS1_DATA_TSID	3
#define GPS2_DATA_TSID	4


/*
 *	Common functions
 */
int32_t swap_bytes (int32_t v);
void swap_bytes (void* str, size_t sz);
int32_t findStrIndex(uint8_t* str, int32_t st_index, int32_t end_index, char c);
int32_t absolute(int32_t x);

#ifdef DEBUG_MODE
void printString(uint8_t* str, int len);
#endif
#endif//ZSTDLIB_H
