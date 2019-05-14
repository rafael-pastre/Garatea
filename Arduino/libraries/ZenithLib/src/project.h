#ifndef PROJECT_H
#define PROJECT_H

#define DEBUG_MODE
#define PROJ_GPS
#define PROJ_BMP
//#define PROJ_SD
//#define PROJ_LORA

// I2C ADDRESS DEFINITIONS
#define SLAVE_I2C_ADD   8                         // Slave address in I2C bus
#define EEPROM0_I2C_ADD 0x50                      // EEPROM0 address in I2C bus
#define EEPROM1_I2C_ADD 0x51                      // EEPROM1 address in I2C bus
#define EEPROM2_I2C_ADD 0x52                      // EEPROM2 address in I2C bus
#define EEPROM3_I2C_ADD 0x53                      // EEPROM3 address in I2C bus
#define EEPROM4_I2C_ADD 0x54                      // EEPROM4 address in I2C bus
#define EEPROM5_I2C_ADD 0x55                      // EEPROM5 address in I2C bus
#define EEPROM6_I2C_ADD 0x56                      // EEPROM6 address in I2C bus
#define EEPROM7_I2C_ADD 0x57                      // EEPROM7 address in I2C bus

// GPS DEFINITIONS
#define GPS_TX          10                        // GPS Serial TX pin corresponding to the arduino pinout
#define GPS_RX          11                        // GPS Serial RX pin corresponding to the arduino pinout
#define GPS_ENABLE      12                        // GPS Serial Enable pin corresponding to the arduino pinout
#define GPS_BAUD_RATE   9600                      // GPS Serial baud-rate
#define GPS_BUFF_LEN    82                        // Maximum lenght of a NMEA message stored on GPS Buffer

// SD DEFINITIONS
#define SD_CS_PIN 4
#define SD_FILE_NAME "test2.txt"

#endif//PROJECT_H