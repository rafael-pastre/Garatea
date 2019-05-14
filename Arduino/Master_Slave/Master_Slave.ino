/*******************************************************************************
                                INCLUDES & DEFINES
*******************************************************************************/
// ARDUINO INCLUDES
#include <Wire.h>                                 // I2C Library
#include <zstdlib.h>

// I2C ADDRESS DEFINITIONS
#define SLAVE_I2C_ADD 8
#define EEPROM0_I2C_ADD 0x50                      
#define EEPROM1_I2C_ADD 0x51                      // first address = 0
#define EEPROM2_I2C_ADD 0x52                      // final address = 
#define EEPROM3_I2C_ADD 0x53
#define EEPROM4_I2C_ADD 0x54
#define EEPROM5_I2C_ADD 0x55
#define EEPROM6_I2C_ADD 0x56
#define EEPROM7_I2C_ADD 0x57

// ERROR HANDLING & DEBUG DEFINITIONS
#define DEBUG_BAUD      115200

// TRASMISSION STATE IDs
// transmissionState defines what information the slave will send to master
// must set transmissionState before every read call on the slave
#define PRESSURE_ID 0
#define TEMPERATURE_ID 1
#define ALTITUDE_ID 2

// 32K x 8 (256Kbit)
// first address = 0
// final address = 32.767‬
//#define MIS_NUMB_ADDR 32

/*******************************************************************************
                                GLOBAL VARIABLES
*******************************************************************************/
int16_t EEPROM_add = 0;
int32_t pressure, temperature, alt;
byte eein = 0;
byte eeout;
uint8_t mission_number;


/*******************************************************************************
                                    MASTER
*******************************************************************************/
void setup() {
  Wire.begin();               // join i2c bus (address optional for master)
  Serial.begin(DEBUG_BAUD);   // start serial for output
  //writeEEPROM(EEPROM0_I2C_ADD, 0, 1);
  //eeout = readEEPROM(EEPROM0_I2C_ADD, 0);
  Serial.println(eeout, DEC);
}

void loop() {
  readBMP(&pressure, &temperature, &alt);
  //writeEEPROM(EEPROM0_I2C_ADD, 516, eein++);
  //writeEEPROM(EEPROM0_I2C_ADD, 0, eein++);
  eeout = readEEPROM(EEPROM7_I2C_ADD, 516);
  Serial.println(eeout, DEC);
  Serial.println(eeout, BIN);
  delay(5000);
}

// Obs: device_add must be between EEPROM0_I2C_ADD and EEPROM7_I2C_ADD
void writeEEPROM(int device_add, int16_t w_add, byte data){
  byte i2c_send;
  Wire.beginTransmission(device_add);
  //Serial.println(w_add, BIN);
  i2c_send = w_add >> 8;
  //Serial.print(i2c_send, BIN);
  //Serial.print(" ");
  Wire.write(i2c_send);
  i2c_send = (byte)w_add;
  //Serial.println(i2c_send, BIN);
  Wire.write(i2c_send);
  Wire.write(data);
  Wire.endTransmission();
  delay(8);
}

byte readEEPROM(int device_add, int16_t w_add){
  byte i2c_send;
  Wire.beginTransmission(device_add);
  i2c_send = w_add >> 8;
  Wire.write(i2c_send);
  i2c_send = (byte)w_add;
  Wire.write(i2c_send);
  Wire.endTransmission();

  Wire.requestFrom(device_add,1);
  if (Wire.available())
    i2c_send = Wire.read();

  return i2c_send;
}
////////////////////////////////////
// BMP180 Communication Functions //
////////////////////////////////////

// Complete Component Reading
// Obs: temperature and altitude ar multiplied by 100
void readBMP(int32_t* pressure, int32_t* temperature, int32_t* alt){
  *pressure = readPressure();
  *temperature = readTemp();
  *alt = readAltitude();

  Serial.print("Pressao: ");
  Serial.print(*pressure);
  Serial.print(" Pa, Binario: ");
  Serial.println(*pressure, BIN);
  Serial.print("Temperatura: ");
  Serial.print(*temperature);
  Serial.print(" C, Binario: ");
  Serial.println(*temperature, BIN);
  Serial.print("Altitude: ");
  Serial.print(*alt);
  Serial.print(" m, Binario: ");
  Serial.println(*alt, BIN);
}

// returns BMP180's pressure
int32_t readPressure(){
  setSlaveTS(PRESSURE_ID);
  return i2c_read_int32();
}

// returns BMP180's temperature*100
int32_t readTemp(){
  setSlaveTS(TEMPERATURE_ID);
  return i2c_read_int32();
}

// returns BMP180's altitude*100
int32_t readAltitude(){
  setSlaveTS(ALTITUDE_ID);
  return i2c_read_int32();
}

///////////////////////////////////
// Basic Communication Functions //
///////////////////////////////////
void setSlaveTS(byte transmissionState){
  Wire.beginTransmission(SLAVE_I2C_ADD);
  Wire.write(transmissionState);
  Wire.endTransmission();
}
