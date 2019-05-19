/*******************************************************************************
                                INCLUDES & DEFINES
*******************************************************************************/

// I2C ADDRESS DEFINITIONS
#define SLAVE_I2C_ADD 8
#define EEPROM_I2C_ADD 0x50 
#define SETUP_TIME 10000

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
#ifdef PROJ_BMP
  zBMPdata bmp_data;
#endif
#ifdef PROJ_GPS1
  zGPS gps1(10, 11, 12, 9600, 82);
#endif
#ifdef PROJ_GPS2
  zGPS gps2(3, 4, 9600, 82);
#endif

void setup() {
  Wire.begin();               // join i2c bus (address optional for master)
  #if defined DEBUG_MODE || defined DEBUG_MODE_HIGH
    Serial.begin(DEBUG_BAUD);   // start serial for output
    Serial.println("Debug Mode");
  #endif
  delay(SETUP_TIME);
  #ifdef PROJ_BMP
    i2c_read_bmp_cal();
  #endif
  //writeEEPROM(0, 1);
  //eeout = readEEPROM(0);
  //Serial.println(eeout, DEC);
}

void loop() {
  
  //i2c_read32();
  //i2c_read(4);
  //bmp_data.raw_pressure = readPressure();
  
  //bmp_data.raw_temperature = readTemp();
  readSensors();

  writeEEPROM(eein, eein);
  eeout = readEEPROM(eein++);
  
  #ifdef DEBUG_MODE_HIGH
    #ifdef PROJ_BMP
      Serial.print("Raw pressure: ");Serial.print(bmp_data.raw_pressure,BIN);Serial.print(", Raw temp: ");Serial.println(bmp_data.raw_temperature,BIN);
      Serial.print("Converted pressure: ");Serial.print(bmp_data.convertPressure());Serial.print(" Converted temp: ");Serial.println(bmp_data.convertTemperature());
      Serial.print("Sea Level Pressure: ");Serial.print(bmp_data.convertSealevelPressure());Serial.print(" Altitude: ");Serial.println(bmp_data.convertAltitude());Serial.println();
    #endif
    #ifdef PROJ_EEPROM
      Serial.print("EEPROM test: ");Serial.print(eeout, DEC);Serial.print(" binary: ");Serial.println(eeout, BIN);
    #endif
    delay(DEBUG_DELAY);
  #endif
  //readBMP(&pressure, &temperature, &alt);
  
  
  //writeEEPROM(0, eein++);
  
  
  //delay(5000);
}

void readSensors(){
  uint32_t u32;
  uint16_t u16;
  
  setSlaveTS(BMP_PRES_TSID);
  u32 = (uint32_t)i2c_read(4);
  if(u32 && u32 != 0x00FFFFFF)
    bmp_data.raw_pressure = u32;
    
  setSlaveTS(BMP_TEMP_TSID);
  u16 = (uint16_t)i2c_read(2);
  if(u16 && u16 != 0xFF)
    bmp_data.raw_temperature = u16;
}

#ifdef PROJ_BMP
int32_t micro_read(){
  byte i2c_recieved;
  int32_t ret = 0;
  uint8_t i = 0;

  while(Wire.available() && i < 2){
    i2c_recieved = Wire.read();
    ret = ret << 8;
    ret += i2c_recieved;
    i++;
    #ifdef DEBUG_BMP_CAL
      Serial.print(i2c_recieved,BIN);Serial.print(" ");
    #endif
  }
  #ifdef DEBUG_BMP_CAL
    Serial.println(" ");Serial.print("Final value: ");Serial.print(ret,DEC);Serial.print("   ");Serial.print("Binary form: ");Serial.println(ret,BIN);
  #endif
  return ret;
}

void i2c_read_bmp_cal(){
  #ifdef DEBUG_BMP_CAL
    Serial.println("Requesting BMP calibration data:");
    Serial.println("Bytes recieved: ");
  #endif
  setSlaveTS(BMP_CAL_TSID);
  Wire.requestFrom(SLAVE_I2C_ADD, 22);    // request 22 bytes from slave (device #8)
  bmp_data.set_ac1((int16_t)micro_read());
  bmp_data.set_ac2((int16_t)micro_read());
  bmp_data.set_ac3((int16_t)micro_read());
  bmp_data.set_ac4((uint16_t)micro_read());
  bmp_data.set_ac5((uint16_t)micro_read());
  bmp_data.set_ac6((uint16_t)micro_read());
  bmp_data.set_b1((int16_t)micro_read());
  bmp_data.set_b2((int16_t)micro_read());
  bmp_data.set_mb((int16_t)micro_read());
  bmp_data.set_mc((int16_t)micro_read());
  bmp_data.set_md((int16_t)micro_read());
}
#endif

int32_t i2c_read(int num_bytes){
  byte i2c_recieved;
  int32_t ret = 0;
  uint8_t i = 0;
  #ifdef DEBUG_MODE
    Serial.print("Requesting ");Serial.print(num_bytes, DEC);Serial.println(" bytes from slave:");
    Serial.println("Bytes recieved: ");
  #endif
  Wire.requestFrom(SLAVE_I2C_ADD, num_bytes);    // request "num_bytes" bytes from slave (device #8)
  while(Wire.available() && i < num_bytes){
    i2c_recieved = Wire.read();
    ret = ret << 8;
    ret += i2c_recieved;
    i++;
    #ifdef DEBUG_MODE
      Serial.print(i2c_recieved,BIN);Serial.print(" ");
    #endif
  }
  #ifdef DEBUG_MODE
    Serial.println(" ");
    Serial.print("Final value: ");Serial.println(ret,DEC);
    Serial.print("Binary form: ");Serial.println(ret,BIN);
  #endif
  return ret;
}

int32_t i2c_read32(){
  byte i2c_recieved;
  int32_t ret = 0;
  uint8_t i = 0;
  #ifdef DEBUG_MODE
    Serial.println("Requesting 4bytes (32 bits) from slave:");
    Serial.println("Bytes recieved: ");
  #endif
  Wire.requestFrom(SLAVE_I2C_ADD, 4);    // request 4 bytes from slave (device #8)
  while(Wire.available() && i < 4){
    i2c_recieved = Wire.read();
    ret = ret << 8;
    ret += i2c_recieved;
    i++;
    #ifdef DEBUG_MODE
      Serial.print(i2c_recieved,BIN);Serial.print(" ");
    #endif
  }
  #ifdef DEBUG_MODE
    Serial.println(" ");
    Serial.print("Final value: ");Serial.println(ret,DEC);
    Serial.print("Binary form: ");Serial.println(ret,BIN);
  #endif
  return ret;
}

// Obs: device_add must be between EEPROM0_I2C_ADD and EEPROM7_I2C_ADD
void writeEEPROM(int16_t w_add, byte data){
  byte i2c_send;
  Wire.beginTransmission(EEPROM_I2C_ADD);
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

byte readEEPROM(int16_t w_add){
  byte i2c_send;
  Wire.beginTransmission(EEPROM_I2C_ADD);
  i2c_send = w_add >> 8;
  Wire.write(i2c_send);
  i2c_send = (byte)w_add;
  Wire.write(i2c_send);
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_I2C_ADD,1);
  if (Wire.available())
    i2c_send = Wire.read();

  return i2c_send;
}

void readSlave(){
  
}
////////////////////////////////////
// BMP180 Communication Functions //
////////////////////////////////////
// returns BMP180's pressure
uint32_t readPressure(){
  setSlaveTS(BMP_PRES_TSID);
  return (uint32_t)i2c_read(4);
}

// returns BMP180's temperature*100
uint16_t readTemp(){
  setSlaveTS(BMP_TEMP_TSID);
  return (uint16_t)i2c_read(2);
}
/**
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
  return i2c_read32();
}

// returns BMP180's temperature*100
int32_t readTemp(){
  setSlaveTS(TEMPERATURE_ID);
  return i2c_read32();
}

// returns BMP180's altitude*100
int32_t readAltitude(){
  setSlaveTS(ALTITUDE_ID);
  return i2c_read32();
}
*/
///////////////////////////////////
// Basic Communication Functions //
///////////////////////////////////
void setSlaveTS(byte transmissionState){
  Wire.beginTransmission(SLAVE_I2C_ADD);
  Wire.write(transmissionState);
  Wire.endTransmission();
}
