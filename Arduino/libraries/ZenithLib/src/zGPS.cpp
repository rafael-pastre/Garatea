#include "zGPS.h"

// GPS GLOBAL DATA VARIABLES
SoftwareSerial gpsSerial(GPS_TX, GPS_RX);         // defines serial(UART) communication with GPS
int32_t GPS_hour=0, GPS_min=0, GPS_sec=0;         // time
int32_t GPS_lat_int=0, GPS_lat_frac=0;            // latitude
char    GPS_NS;                                   // north/south
int32_t GPS_long_int=0, GPS_long_frac=0;          // longitude
char    GPS_EW;                                   // east/west
char    GPS_quality='0';                          // quality
int32_t GPS_Satellites=0;                         // number of satelites
int32_t GPS_HDOP_int=0, GPS_HDOP_frac=0;          // horizontal precision
int32_t GPS_Altitude=0;                           // altitude
int32_t GPS_age_int=0, GPS_age_frac=0;            // information age

// PRIVATE VARIABLES
byte    GPSBuffer[GPS_BUFF_LEN];                  // buffer for NMEA comunication
size_t  GPSBufferLength=0;                        // stores the length of the GPSBuffer after 'checkGPS'

// NMEA MESSAGE PROCESSING
int32_t NMEADiv[NMEA_DIV_LEN];                    // position of the GPSBuffer terms
size_t  NMEADivLen=0;                             // curent number of terms(divisions) in the GPSBuffer. limited by NMEA_SUB_LEN
char    NMEATalkerID[3];                          // sender of NMEA message
char    NMEAMnemonic[4];                          // mnemonic for message type

// PRIVATE FUNCTIONS
err_t readStringBuffer(char* str, int32_t st_index, int32_t N){
  // this function stores values from the GPSBuffer in the "str" string
  // reads "N" ascii characters, starting from "st_index"
  if(N < 0)
    return INDEX_ERR;

  for(int i = 0; i < N; i++){
    str[i] = GPSBuffer[st_index+i];
  }

  str[N] = '\0';
  return NO_ERR;
}

err_t readBufferIndex(int32_t st_index, int32_t end_index, char ignore, int32_t* val_addr){
  // this function reads a integer value from the buffer and stores it in val_addr
  // each digit of the value read must be coded in ascii format
  // "st_index" is the starting index and "end_index" is the ending index for reading
  // "ignore" characters will be ignored in reading
  // characters in "st_index" and "end_index" will be read
  int32_t aux;
  if(st_index > end_index)
    return INDEX_ERR;
    
  aux = 0;
  
  while(st_index <= end_index){
    if(GPSBuffer[st_index] == ignore){
      st_index++;
      continue;
    }
    if ((GPSBuffer[st_index] < '0') || (GPSBuffer[st_index] > '9'))
      return READ_ERR;

    aux = aux*10;
    aux += GPSBuffer[st_index] - '0';
    st_index++;
  }

  if(aux <= 0)
    return INV_DATA_ERR;

  *val_addr = aux;
  return NO_ERR;
}

void setNMEADiv(char separator){
  NMEADivLen = 0;
  for(int32_t i = 0; i < GPSBufferLength; i++)
    if(GPSBuffer[i] == separator)
      NMEADiv[NMEADivLen++] = i;
}

err_t ProcessGGA(){
  // Processes a NMEA message with "GGA" Mnemonic and attributes value for corresponding global variables
  // this function is called by "GPS_Process_NMEA_Line"
  // the message must be in the GPSBuffer before this functions calling
  int32_t aux;
  err_t err_return = NO_ERR;
  setNMEADiv(',');

  //printString(GPSBuffer, GPSBufferLength);
  //TO-DO: check an handle error in each read
  // time (NMEADiv[0]~NMEADiv[1])
  readBufferIndex(NMEADiv[0]+1, NMEADiv[0]+2, NMEA_IGNR_NULL, &GPS_hour);         // hour (2 chars)
  readBufferIndex(NMEADiv[0]+3, NMEADiv[0]+4, NMEA_IGNR_NULL, &GPS_min);          // minutes (2 chars)
  readBufferIndex(NMEADiv[0]+5, NMEADiv[1]-1, '.', &GPS_sec);                     // seconds (ignore '.')
  
  // latitude (NMEADiv[1]~NMEADiv[2])
  readBufferIndex(NMEADiv[1]+1, NMEADiv[1]+2, NMEA_IGNR_NULL, &GPS_lat_int);      // integer part (2 chars)
  readBufferIndex(NMEADiv[1]+3, NMEADiv[2]-1, '.', &GPS_lat_frac);                // fractional part (ignore '.')

  // N/S (NMEADiv[2]~NMEADiv[3])
  if ((GPSBuffer[NMEADiv[2]+1] == 'N') || (GPSBuffer[NMEADiv[2]+1] == 'S'))
    GPS_NS = GPSBuffer[NMEADiv[2]+1];
  else
    err_return = INV_DATA_ERR;
      
  // longitude (NMEADiv[3]~NMEADiv[4])
  readBufferIndex(NMEADiv[3]+1, NMEADiv[3]+3, NMEA_IGNR_NULL, &GPS_long_int);     // integer part (3 chars)
  readBufferIndex(NMEADiv[3]+4, NMEADiv[4]-1, '.', &GPS_long_frac);               // fractional part (ignore '.')

  // E/W (NMEADiv[4]~NMEADiv[5])
  if ((GPSBuffer[NMEADiv[4]+1] == 'E') || (GPSBuffer[NMEADiv[4]+1] == 'W'))
    GPS_EW = GPSBuffer[NMEADiv[4]+1];
  else
    err_return = INV_DATA_ERR;

  // quality (NMEADiv[5]~NMEADiv[6])
  if ((GPSBuffer[NMEADiv[5]+1] >= '0') && (GPSBuffer[NMEADiv[5]+1] <= '9'))
    GPS_quality = GPSBuffer[NMEADiv[5]+1];
  else
    err_return = INV_DATA_ERR;
    
  // numSat (NMEADiv[6]~NMEADiv[7])
  readBufferIndex(NMEADiv[6]+1, NMEADiv[7]-1, NMEA_IGNR_NULL, &GPS_Satellites);

  // horizontal precision(HDOP) (NMEADiv[7]~NMEADiv[8])
  aux = findStrIndex(GPSBuffer, NMEADiv[7]+1, NMEADiv[8]-1, '.');
  if(aux > 0){
    readBufferIndex(NMEADiv[7]+1, aux-1, NMEA_IGNR_NULL, &GPS_HDOP_int);
    readBufferIndex(aux+1, NMEADiv[8]-1, NMEA_IGNR_NULL, &GPS_HDOP_frac);
  }  
  else
    err_return = INV_DATA_ERR;

  // altitude (NMEADiv[8]~NMEADiv[9])
  aux = findStrIndex(GPSBuffer, NMEADiv[8]+1, NMEADiv[9]-1, '.');
  if(aux > 0){
    readBufferIndex(NMEADiv[8]+1, aux-1, NMEA_IGNR_NULL, &GPS_Altitude);
    //readBufferIndex(aux+1, NMEADiv[9]-1, NMEA_IGNR_NULL, &GPS_HDOP_frac);
  }  
  else
    err_return = INV_DATA_ERR;

  //GPS_print_Info();
  return err_return;
}

// USER FUNCTIONS
err_t GPS_update(){
  // Reads a NMEA message from GPS and stores it in GPSBuffer
  int inByte;
  int32_t aux = 0;
  
  while (gpsSerial.available() > 0){
    if(aux >= GPS_BUFF_LEN)
      return READ_ERR;

    inByte = gpsSerial.read();
    if (inByte != '$' && !aux)
      continue;
    
	#ifdef DEBUG_MODE
    Serial.write(inByte);                         // Output raw NMEA string from GPS.
    #endif
	
    if (inByte == '$')
      aux = 0;
    if (inByte != '\r')
      GPSBuffer[aux++] = inByte;
    if (inByte == '\n'){
      GPSBufferLength = aux;
      return NO_ERR;
    }
  }
}

err_t GPS_Process_NMEA_Line(){
  // Read NMEA message TalkerId and Mnemonic and process it accordingly
  // the message must be in the GPSBuffer before this functions calling
  //
  // NMEA talker ID reference table:
  //  ______________________
  // |    GNSS     |TalkerID|
  // |-------------|--------|
  // |GPS,SBAS,QZSS|   GP   |
  // |   GLONASS   |   GL   |
  // |   Galileo   |   GA   |
  // |   BeiDou    |   GB   |
  // |    any      |   GN   |
  //  ----------------------
  readStringBuffer(NMEATalkerID, 1, 2);           // get NMEA talker ID
  readStringBuffer(NMEAMnemonic, 3, 3);           // get NMEA mnemonic that defines message parameters
  //Serial.println(NMEAMnemonic);
  if(!strcmp(NMEAMnemonic,"GGA"))                 // process NMEA message with "GGA" Mnemonic (implementar outras mensagens)
    ProcessGGA();
  else
    return INV_DATA_ERR;
  
  return NO_ERR;
}

err_t GPS_init(){
  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, HIGH);
  // TO-DO: protect init
  gpsSerial.begin(GPS_BAUD_RATE);
  return NO_ERR;
}

err_t GPS_read(){
  err_t error_value;
  error_value = GPS_update();
  if(error_value != NO_ERR)
    return error_value;
  error_value = GPS_Process_NMEA_Line();
  return error_value;
}

// DEBUG
