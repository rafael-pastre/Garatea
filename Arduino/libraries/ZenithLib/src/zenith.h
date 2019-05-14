#ifndef ZENITH_H
#define ZENITH_H

//#include "zdebug.h"
#include "zstdlib.h"

#ifdef PROJ_BMP
#include "zBMP.h"                                 // BMP Library
#endif
#ifdef PROJ_GPS
#include "zGPS.h"                                 // GPS Library
#endif
#ifdef PROJ_LORA
#include "zLoRa.h"                                // LoRa Library
#endif
#ifdef PROJ_SD
#include "zSD.h"                                  // SD Library
#endif
#ifdef PROJ_EEPROM
#include "zEEPROM.h"                              // EEPROM Library
#endif

//#include "zMS.h"                                  // Master-Slave Library


#ifdef DEBUG_MODE
#define DEBUG_BAUD      115200                    // PC Serial baud rate for debug

// General
void printString(byte* str, int len);

// BMP
#ifdef PROJ_BMP

#endif//PROJ_BMP

// GPS
#ifdef PROJ_GPS
void GPS_print_Altitude();
void GPS_print_Satellites();
void GPS_print_Time();
void GPS_print_Latitude();
void GPS_print_Longitude();
void GPS_print_Quality();
void GPS_print_HDOP();
void GPS_print_Age();
void GPS_print_Info();
#endif//PROJ_GPS
#endif

#endif//ZENITH_H