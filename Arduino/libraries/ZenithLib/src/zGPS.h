#ifndef ZGPS_H
#define ZGPS_H

#include "zstdlib.h"

// GPS DEFAULT DEFINITIONS
#ifndef GPS_TX
#define GPS_TX          10                        // GPS Serial TX pin corresponding to the arduino pinout
#endif
#ifndef GPS_RX
#define GPS_RX          11                        // GPS Serial RX pin corresponding to the arduino pinout
#endif
#ifndef GPS_ENABLE
#define GPS_ENABLE      12                        // GPS Serial Enable pin corresponding to the arduino pinout
#endif
#ifndef GPS_BAUD_RATE
#define GPS_BAUD_RATE   9600                      // GPS Serial baud-rate
#endif
#ifndef GPS_BUFF_LEN
#define GPS_BUFF_LEN    82                        // Maximum lenght of a NMEA message stored on GPS Buffer
#endif

// NMEA INTERPRETER DEFINITIONS
#define NMEA_DIV_LEN    20                        // Maximum number of parameters of a NMEA message (Lenght of the vector that stores the index of each parameter of the NMEA message in the GPSBuffer)
#define NMEA_IGNR_NULL  '\0'                      // Default "ignore" character for "readBufferIndex" function for not ignoring any character

// GPS GLOBAL DATA VARIABLES
extern SoftwareSerial gpsSerial;                  // defines serial(UART) communication with GPS
extern int32_t GPS_hour, GPS_min, GPS_sec;        // time
extern int32_t GPS_lat_int, GPS_lat_frac;         // latitude
extern char    GPS_NS;                            // north/south
extern int32_t GPS_long_int, GPS_long_frac;       // longitude
extern char    GPS_EW;                            // east/west
extern char    GPS_quality;                       // quality
extern int32_t GPS_Satellites;                    // number of satelites
extern int32_t GPS_HDOP_int, GPS_HDOP_frac;       // horizontal precision
extern int32_t GPS_Altitude;                      // altitude
extern int32_t GPS_age_int, GPS_age_frac;         // information age

// USER FUNCTIONS
err_t GPS_init();
err_t GPS_read();
err_t GPS_update();
err_t GPS_Process_NMEA_Line();
#endif//ZGPS_H