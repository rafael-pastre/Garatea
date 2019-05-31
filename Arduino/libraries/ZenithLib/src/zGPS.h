#ifndef ZGPS_H
#define ZGPS_H

#include "zstdlib.h"

// NMEA INTERPRETER DEFINITIONS
#define ALWAYS_ENABLED	0xFF
#define GPS_UPDATE_TIME	1000
#define NMEA_DIV_LEN    20                        // Maximum number of parameters of a NMEA message (Lenght of the vector that stores the index of each parameter of the NMEA message in the GPSBuffer)
#define NMEA_IGNR_NULL  '\0'                      // Default "ignore" character for "readBufferIndex" function for not ignoring any character

class zGPS{
	public:
		zGPS(uint8_t tx, uint8_t rx, uint32_t baud, uint16_t buff_len);
		zGPS(uint8_t tx, uint8_t rx, uint8_t en, uint32_t baud, uint16_t buff_len);
		void init();
		err_t read();

    // Airborne mode
    void setGPS_DynamicMode6();
    //data variables
    int32_t date;
    int32_t Time;
    int32_t lat;
    int32_t lon;
    int32_t numSat;
    int32_t Hdop;
    int32_t alt;
    int32_t age;
    int32_t spd;

    uint8_t NS, EW;
    
	private:
		// project variables (cannot be changed after defining the sensor)
		uint8_t GPS_TX;							// TX pin for serial communication
		uint8_t GPS_RX;							// RX pin for serial communication
		uint8_t GPS_ENABLE;						// Enable pin for serial communication
		uint32_t GPS_BAUD_RATE;					// Baud rate for serial communication
		uint8_t GPS_BUFF_LEN;					// Size of NMEA message buffer

		// communication variables
		SoftwareSerial gpsSerial;                  // defines serial(UART) communication with GPS
		
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
		
		// NMEA auxiliar methods
		err_t readStringBuffer(char* str, int32_t st_index, int32_t N);
    int32_t readParameter(uint8_t par_id);
		void setNMEADiv(char separator);
		void ProcessGGA();
    void ProcessRMC();
    void ProcessGLL();
    void ProcessGNS();

    // UBX communication (Airborne mode auxiliar)
    void sendUBX(uint8_t *MSG, uint8_t len);
    boolean getUBX_ACK(uint8_t *MSG);
   
};

#endif//ZGPS_H