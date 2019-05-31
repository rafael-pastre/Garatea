/****************************************************************************************
 *	Reference for project macros:
 *
 *	Components macros:
 *	USING_BMP		-- defined if the project uses on-board BMP sensor
 *	USING_GPS1		-- defined if the project uses GPS on the right port
 *	USING_GPS2		-- defined if the project uses GPS on the left port
 *	USING_DHT		-- defined if the project uses DHT on the right port
 *	USING_LORA		-- defined if the project uses on-board LoRa communication module
 *	USING_EEPROM		-- defined if the project uses on-board EEPROM
 *	USING_SD			-- defined if the project uses external SD card
 *
 *	Serial output macros:
 *	LAUNCH_MODE      -- outputs information for lauchment
 *	DEBUG_UBX        -- outputs information of UBX communication
 *	DEBUG_I2C		-- outputs information of I2C communication
 *	DEBUG_NMEA		-- outputs information of all NMEA communication and processing
 *	 DEBUG_NMEA_CS	-- outputs information of NMEA checksum
 *	DEBUG_LOGIC		-- outputs logical information of functions
 ***************************************************************************************/

/*
 *	Components options
 */
#define USING_BMP
#define USING_GPS1
//#define USING_GPS2

/*
 *	Output options
 */
#define DEBUG_MODE
#define DEBUG_MODE_HIGH
#define DEBUG_NMEA_CS
//#define DEBUG_GPS
