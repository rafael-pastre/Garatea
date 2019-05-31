#include "zstdlib.h"
//#include <SoftwareSerial.h>
// NMEA INTERPRETER DEFINITIONS
#define ALWAYS_ENABLED  0xFF
#define GPS_UPDATE_TIME 5000
#define NMEA_DIV_LEN    20                        // Maximum number of parameters of a NMEA message (Lenght of the vector that stores the index of each parameter of the NMEA message in the GPSBuffer)
#define NMEA_IGNR_NULL  '\0'                      // Default "ignore" character for "readBufferIndex" function for not ignoring any character


typedef struct {
  // project variables (cannot be changed after defining the sensor)
  uint8_t tx;
  uint8_t rx;
  uint8_t en;
  uint8_t buff_len;

  // communication variables
  SoftwareSerial gpsSerial;                  // defines serial(UART) communication with GPS
    
  //data variables
  int32_t GPS_hour, GPS_min, GPS_sec;        // time
  int32_t GPS_lat_int, GPS_lat_frac;         // latitude
  char    GPS_NS;                            // north/south
  int32_t GPS_long_int, GPS_long_frac;       // longitude
  char    GPS_EW;                            // east/west
  char    GPS_quality;                       // quality
  int32_t GPS_Satellites;                    // number of satelites
  int32_t GPS_HDOP_int, GPS_HDOP_frac;       // horizontal precision
  int32_t GPS_Altitude;                      // altitude
  int32_t GPS_age_int, GPS_age_frac;         // information age
    
  // PRIVATE VARIABLES
  byte*   GPSBuffer;                  // buffer for NMEA comunication
  size_t  GPSBufferLength;                          // stores the length of the GPSBuffer after 'checkGPS'

  // NMEA MESSAGE PROCESSING
  uint8_t NMEADiv[NMEA_DIV_LEN];                    // position of the GPSBuffer terms
  size_t  NMEADivLen;                               // curent number of terms(divisions) in the GPSBuffer. limited by NMEA_SUB_LEN
  char    NMEATalkerID[3];                          // sender of NMEA message
  char    NMEAMnemonic[4]; 
}GPS;

void GPS_init(GPS* gps,  uint8_t tx, uint8_t rx, uint8_t en, uint8_t buff_len){
  
}

GPS* gps1;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
