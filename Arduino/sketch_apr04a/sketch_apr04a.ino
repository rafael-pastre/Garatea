//#include <zstdlib.h>

#include <Wire.h>                                 // I2C Library
#include <SoftwareSerial.h>                       // UART Library
//#include "project.h"
#include <stdlib.h>

#define DEBUG_MODE
#define DEBUG_MODE_HIGH
#define DEBUG_BAUD   115200
#define DEBUG_DELAY 5000

// ERROR HANDLING (do not comment after err_t or it will cause compilation error) may be deprecated with Object Orientation
#define err_t           uint8_t
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


// NMEA INTERPRETER DEFINITIONS
#define ALWAYS_ENABLED  0xFF
#define GPS_UPDATE_TIME 5000
#define NMEA_DIV_LEN    20                        // Maximum number of parameters of a NMEA message (Lenght of the vector that stores the index of each parameter of the NMEA message in the GPSBuffer)
#define NMEA_IGNR_NULL  '\0'                      // Default "ignore" character for "readBufferIndex" function for not ignoring any character

class zGPS{
  public:
    zGPS(uint8_t tx, uint8_t rx, uint32_t baud, uint16_t buff_len);
    zGPS(uint8_t tx, uint8_t rx, uint8_t en, uint32_t baud, uint16_t buff_len);
    void init();
    err_t read();
    
    uint32_t time();
    int32_t lat();
    int32_t lon();
    
  private:
    // project variables (cannot be changed after defining the sensor)
    uint8_t GPS_TX;             // TX pin for serial communication
    uint8_t GPS_RX;             // RX pin for serial communication
    uint8_t GPS_ENABLE;           // Enable pin for serial communication
    uint32_t GPS_BAUD_RATE;         // Baud rate for serial communication
    uint8_t GPS_BUFF_LEN;         // Size of NMEA message buffer

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
    char    NMEAMnemonic[4];                          // mnemonic for message type

    // message processing methods
    err_t GPS_update();
    err_t GPS_Process_NMEA_Line();
    
    // auxiliar methods
    err_t readStringBuffer(char* str, int32_t st_index, int32_t N);
    err_t readBufferIndex(int32_t st_index, int32_t end_index, char ignore, int32_t* val_addr);
    void zGPS::setNMEADiv(char separator);
    err_t zGPS::ProcessGGA();
};

//int32_t i2c_read_int32();
int32_t swap_bytes (int32_t v);
void swap_bytes (void* str, size_t sz);
int32_t findStrIndex(byte* str, int32_t st_index, int32_t end_index, char c);

#ifdef DEBUG_MODE
void printString(byte* str, int len);
#endif

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

}

void loop() { // run over and over
  Serial.write("Hello World");
  delay(1000);
}

int32_t swap_bytes (int32_t v){
  // Swaps the order of bytes in a int32_t variable:
  // Example: int32_t in  = b3,b2,b1,b0 -> int32_t out = b0,b1,b2,b3
  int32_t ret = 0;
  byte b;
  for(int i = 0; i < 4; i++){
    ret = ret << 8;
    b = v;
    ret += b;
    v = v >> 8;
  }
  return ret;
}

void swap_bytes (void* str, size_t sz){
  // Example: int32_t in  = b3,b2,b1,b0 -> int32_t out = b0,b1,b2,b3
  uint8_t b;
  for(size_t i = 0; i < sz/2; i++){
        b = *((uint8_t*)(str+sz-i-1));
        *((uint8_t*)(str+sz-i-1)) = *((uint8_t*)(str+i));
    *((uint8_t*)(str+i)) = b;
  }
}

int32_t findStrIndex(byte* str, int32_t st_index, int32_t end_index, char c){
  // Finds the and returns the index of the first ocurrence of ascii char "c" in "str" string
  // the search begins in "st_index" and ends in "end_index"
  // "st_index" and "end_index" are included in the search
  int32_t i;
  for(i = st_index; i <= end_index; i++)
    if(str[i] == c)
      return i;
  
  return -1;
}

// DEBUG
void printString(byte* str, int len){
  for(int i = 0; i < len; i++)
    Serial.write(str[i]);
}
