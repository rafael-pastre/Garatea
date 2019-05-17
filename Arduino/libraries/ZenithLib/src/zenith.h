#ifndef ZENITH_H
#define ZENITH_H

//#include "zdebug.h"
#include "zstdlib.h"

#include "zBMP.h"                                 // BMP Library
#include "zBMPdata.h"
#include "zGPS.h"                                 // GPS Library

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

#endif//ZENITH_H