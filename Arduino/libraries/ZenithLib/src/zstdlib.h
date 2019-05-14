#ifndef ZSTDLIB_H
#define ZSTDLIB_H

#if ARDUINO >= 100                                // Arduino Default Library
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>                                 // I2C Library
#include <SoftwareSerial.h>                       // UART Library
#include "project.h"

// ERROR HANDLING (do not comment after err_t or it will cause compilation error)
#define err_t           int32_t
#define NO_ERR          0                         // sucessful execution
#define INDEX_ERR       1                         // index error
#define READ_ERR        2                         // read error
#define INV_DATA_ERR    3                         // invalid data
#define TRANSMISS_ERR   4                         // transmission error
#define INIT_ERR        5                         // initialization error

void i2c_send(int32_t v);
int32_t i2c_read_int32();
int32_t swap_bytes (int32_t v);
int32_t findStrIndex(byte* str, int32_t st_index, int32_t end_index, char c);

#ifdef DEBUG_MODE
void printString(byte* str, int len);
#endif
#endif//ZSTDLIB_H