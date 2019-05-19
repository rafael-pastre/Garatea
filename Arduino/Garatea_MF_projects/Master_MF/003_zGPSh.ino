#ifndef ZGPS_H
#define ZGPS_H

// NMEA INTERPRETER DEFINITIONS
#define ALWAYS_ENABLED	0xFF
#define GPS_UPDATE_TIME	5000
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
		uint8_t GPS_TX;							// TX pin for serial communication
		uint8_t GPS_RX;							// RX pin for serial communication
		uint8_t GPS_ENABLE;						// Enable pin for serial communication
		uint32_t GPS_BAUD_RATE;					// Baud rate for serial communication
		uint8_t GPS_BUFF_LEN;					// Size of NMEA message buffer

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

#endif//ZGPS_H
