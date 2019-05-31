/*******************************************************************************
                                GLOBAL VARIABLES
*******************************************************************************/
// I2C ADDRESS DEFINITIONS
#define SLAVE_I2C_ADD 8
#define SETUP_TIME 5000

#define SD_CS_PIN 4
#define SD_FILE_NAME "test.txt"

#define LORA_CS_PIN 7
#define LORA_RST_PIN A0
#define LORA_DIO0_PIN 2

#define DEBUG_MODE_I2C
int16_t EEPROM_add = 0;
int16_t iterations = 0;

zPACKET slave_packet;
zEEPROM eeprom;
#ifdef PROJ_SD
File sdFile;
#endif
/*******************************************************************************
                                    SETUP
*******************************************************************************/
void setup() {
  Wire.begin();               // join i2c bus (address optional for master)

  #if defined DEBUG_MODE || defined DEBUG_MODE_HIGH
    Serial.begin(DEBUG_BAUD);   // start serial for output
    Serial.println(F("Debug Mode"));
  #endif

  //writeEEPROM(5, &test, sizeof(uint32_t));
  //test2 = *((uint32_t*)readEEPROM(5, sizeof(uint32_t)));
  
  #ifdef PROJ_EEPROM
    eeprom.init();
  #endif

  #ifdef PROJ_SD
    Serial.println(F("Inicializando cartao SD"));
    while(!SD.begin(SD_CS_PIN)){
      #ifdef DEBUG_MODE
        Serial.println(F("Falha na inicializacao do cartao SD. Tentando novamente."));
      #endif
    }
    #ifdef DEBUG_MODE
      Serial.println(F("Cartao SD inicializado"));
    #endif
  #endif

  #ifdef PROJ_LORA
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  
    if (!LoRa.begin(433.123E6)) {
      Serial.println("Starting LoRa failed!");
      while (1);
    }
    LoRa.setTxPower(20, 1);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setSpreadingFactor(11);
    LoRa.enableCrc();
    Serial.println("LoRa started");
  #endif
  
  delay(SETUP_TIME);
  
  #ifdef PROJ_BMP
    read_bmp_cal();
  #endif
  wdt_enable(WDTO_8S);
}

/*******************************************************************************
                                   LOOP
*******************************************************************************/
void loop() {
  #ifdef DEBUG_MODE
    Serial.print(F("Iteration "));Serial.println(++iterations);
    Serial.println();
  #endif
  
  readSensors();
  
  #ifdef PROJ_EEPROM
    writeLogEEPROM();
  #endif

  #ifdef PROJ_SD
    writeLogSD();
  #endif

  #ifdef PROJ_LORA
    sendLoRa();
  #endif
  
  #ifdef DEBUG_MODE_HIGH
    #ifdef PROJ_BMP
      Serial.println(F("BMP data:"));
      Serial.print(F("Raw pressure: "));Serial.print(slave_packet.BMP_pressure,HEX);Serial.print(F("\t\t Raw temp: "));Serial.println(slave_packet.BMP_temperature,HEX);
      Serial.print(F("Converted pressure: "));Serial.print(slave_packet.convertPressure());Serial.print(F("\t Converted temp: "));Serial.println(slave_packet.convertTemperature());
      Serial.print(F("Sea Level Pressure: "));Serial.print(slave_packet.convertSealevelPressure());Serial.print(F("\t Altitude: "));Serial.println(slave_packet.convertAltitude());
      Serial.println();
    #endif
    #ifdef PROJ_GPS1
      Serial.println(F("GPS 1 data:"));
      Serial.print(F("Date: "));Serial.print(slave_packet.gps1.date);Serial.print(F("\t Time: "));Serial.println(slave_packet.gps1.Time);
      Serial.print(F("Latitude: "));Serial.print(slave_packet.gps1.lat);Serial.print(F("\t Longitude: "));Serial.print(slave_packet.gps1.lon);Serial.print(F("\t Altitude: "));Serial.print(slave_packet.gps1.alt);Serial.print(F("\t Speed: "));Serial.println(slave_packet.gps1.spd);
      Serial.print(F("Satellites: "));Serial.print(slave_packet.gps1.numSat);Serial.print(F("\t Hdop: "));Serial.print(slave_packet.gps1.Hdop);Serial.print(F("\t Info age: "));Serial.println(slave_packet.gps1.age);
      Serial.println();
    #endif
    #ifdef PROJ_GPS2
      Serial.println(F("GPS 2 data:"));
      Serial.print(F("Date: "));Serial.print(slave_packet.gps2.date);Serial.print(F("\t Time: "));Serial.println(slave_packet.gps2.Time);
      Serial.print(F("Latitude: "));Serial.print(slave_packet.gps2.lat);Serial.print(F("\t Longitude: "));Serial.print(slave_packet.gps2.lon);Serial.print(F("\t Altitude: "));Serial.print(slave_packet.gps2.alt);Serial.print(F("\t Speed: "));Serial.println(slave_packet.gps2.spd);
      Serial.print(F("Satellites: "));Serial.print(slave_packet.gps2.numSat);Serial.print(F("\t Hdop: "));Serial.print(slave_packet.gps2.Hdop);Serial.print(F("\t Info age: "));Serial.println(slave_packet.gps2.age);
      Serial.println();
    #endif
    #ifdef PROJ_EEPROM
      Serial.print(F("EEPROM dptr: "));Serial.println(eeprom.dptr);
      Serial.println();
    #endif

/*
    if(iterations == 10)
      readLogEEPROM();
      */
  #endif  

  //if(iterations < 10)
    wdt_reset();
  delay(4500);
}

/*******************************************************************************
                            MASTER-SLAVE FUNCTIONS
*******************************************************************************/
void readSensors(){

  // Request BMP data
  #ifdef PROJ_BMP
    setSlaveTS(BMP_DATA_TSID);
    Wire.requestFrom(SLAVE_I2C_ADD, sizeof(uint32_t) + sizeof(uint16_t));    // request 6 bytes from slave (device #8)
    slave_packet.BMP_pressure = (uint32_t)i2c_read(4);
    slave_packet.BMP_temperature = (uint16_t)i2c_read(2);
  #endif

  // Request GPS1 data
  #ifdef PROJ_GPS1
    setSlaveTS(GPS1_DATA_TSID);
    Wire.requestFrom(SLAVE_I2C_ADD, sizeof(gps_data));    // request 36 bytes from slave (device #8)
    slave_packet.gps1.date = (int32_t)i2c_read(4);
    slave_packet.gps1.Time = (int32_t)i2c_read(4);
    slave_packet.gps1.lat = (int32_t)i2c_read(4);
    slave_packet.gps1.lon = (int32_t)i2c_read(4);
    slave_packet.gps1.numSat = (int32_t)i2c_read(4);
    slave_packet.gps1.Hdop = (int32_t)i2c_read(4);
    slave_packet.gps1.alt = (int32_t)i2c_read(4);
    slave_packet.gps1.spd = (int32_t)i2c_read(4);
    slave_packet.gps1.age = (int32_t)i2c_read(4);
  #endif
  
  // Request GPS2 data
  #ifdef PROJ_GPS2
    setSlaveTS(GPS2_DATA_TSID);
    Wire.requestFrom(SLAVE_I2C_ADD, sizeof(gps_data));    // request 36 bytes from slave (device #8)
    slave_packet.gps2.date = (int32_t)i2c_read(4);
    slave_packet.gps2.Time = (int32_t)i2c_read(4);
    slave_packet.gps2.lat = (int32_t)i2c_read(4);
    slave_packet.gps2.lon = (int32_t)i2c_read(4);
    slave_packet.gps2.numSat = (int32_t)i2c_read(4);
    slave_packet.gps2.Hdop = (int32_t)i2c_read(4);
    slave_packet.gps2.alt = (int32_t)i2c_read(4);
    slave_packet.gps2.spd = (int32_t)i2c_read(4);
    slave_packet.gps2.age = (int32_t)i2c_read(4);
  #endif
}

void read_bmp_cal(){
  #ifdef DEBUG_MODE
    Serial.println(F("Requesting BMP calibration data:"));
    Serial.println(F("Bytes recieved: "));
  #endif
  setSlaveTS(BMP_CAL_TSID);
  Wire.requestFrom(SLAVE_I2C_ADD, 22);    // request 22 bytes from slave (device #8)
  slave_packet.ac1 = (int16_t)i2c_read(2);
  slave_packet.ac2 = (int16_t)i2c_read(2);
  slave_packet.ac3 = (int16_t)i2c_read(2);
  slave_packet.ac4 = (uint16_t)i2c_read(2);
  slave_packet.ac5 = (uint16_t)i2c_read(2);
  slave_packet.ac6 = (uint16_t)i2c_read(2);
  slave_packet.b1 = (int16_t)i2c_read(2);
  slave_packet.b2 = (int16_t)i2c_read(2);
  slave_packet.mb = (int16_t)i2c_read(2);
  slave_packet.mc = (int16_t)i2c_read(2);
  slave_packet.md = (int16_t)i2c_read(2);
  #ifdef DEBUG_MODE
    Serial.println(F("Finished reading BMP calibration data with values:"));
    Serial.print(F("ac1: "));Serial.print(slave_packet.ac1);Serial.print(F("\t hex: "));Serial.println(slave_packet.ac1,HEX);
      Serial.print(F("ac2: "));Serial.print(slave_packet.ac2);Serial.print(F("\t hex: "));Serial.println(slave_packet.ac2,HEX);
      Serial.print(F("ac3: "));Serial.print(slave_packet.ac3);Serial.print(F("\t hex: "));Serial.println(slave_packet.ac3,HEX);
      Serial.print(F("ac4: "));Serial.print(slave_packet.ac4);Serial.print(F("\t hex: "));Serial.println(slave_packet.ac4,HEX);
      Serial.print(F("ac5: "));Serial.print(slave_packet.ac5);Serial.print(F("\t hex: "));Serial.println(slave_packet.ac5,HEX);
      Serial.print(F("ac6: "));Serial.print(slave_packet.ac6);Serial.print(F("\t hex: "));Serial.println(slave_packet.ac6,HEX);
      Serial.print(F("b1: "));Serial.print(slave_packet.b1);Serial.print(F("\t hex: "));Serial.println(slave_packet.b1,HEX);
      Serial.print(F("b2: "));Serial.print(slave_packet.b2);Serial.print(F("\t\t hex: "));Serial.println(slave_packet.b2,HEX);
      Serial.print(F("mb: "));Serial.print(slave_packet.mb);Serial.print(F("\t hex: "));Serial.println(slave_packet.mb,HEX);
      Serial.print(F("mc: "));Serial.print(slave_packet.mc);Serial.print(F("\t hex: "));Serial.println(slave_packet.mc,HEX);
      Serial.print(F("md: "));Serial.print(slave_packet.md);Serial.print(F("\t hex: "));Serial.println(slave_packet.md,HEX);
    Serial.println();
  #endif
}

/*******************************************************************************
                                 EEPROM LOG
*******************************************************************************/
void writeLogEEPROM(){
  uint16_t expected_size = 0;

  // Calculation of the expected number of bytes to be written
  #ifdef PROJ_BMP
    expected_size += sizeof(uint32_t) + sizeof(uint16_t);
  #endif
  #ifdef PROJ_GPS1
    expected_size += sizeof(gps_data);
  #endif
  #ifdef PROJ_GPS2
    expected_size += sizeof(gps_data);
  #endif

  // Address overflow treatment
  eeprom.dptr = *((uint16_t*)eeprom.read(DPTR_ADD, sizeof(uint16_t)));
  if(eeprom.dptr + expected_size < eeprom.dptr){                                    // Detects address overflow
    eeprom.dptr = DATA_START_ADD;                                                   // Resets data pointer to start
  }

  // Data saving
  #ifdef PROJ_BMP
    eeprom.write(eeprom.dptr, &slave_packet.BMP_pressure, sizeof(uint32_t));        // Save bmp raw pressure
    eeprom.dptr += sizeof(uint32_t);
    eeprom.write(eeprom.dptr, &slave_packet.BMP_temperature, sizeof(uint16_t));     // Save bmp raw temperature
    eeprom.dptr += sizeof(uint16_t);
  #endif
  #ifdef PROJ_GPS1
    eeprom.write(eeprom.dptr, &slave_packet.gps1, sizeof(gps_data));                // Save gps1 data
    eeprom.dptr += sizeof(gps_data);
  #endif
  #ifdef PROJ_GPS2
    eeprom.write(eeprom.dptr, &slave_packet.gps2, sizeof(gps_data));                // Save gps2 data
    eeprom.dptr += sizeof(gps_data);
  #endif
  
  eeprom.write(DPTR_ADD, &eeprom.dptr, sizeof(uint16_t));                           // Save dptr final value
}

void readLogEEPROM(){
  #ifdef PROJ_BMP
    uint32_t pres;
    uint16_t temp;
  #endif
  #ifdef PROJ_GPS1
    gps_data gps1_tmp;
  #endif
  #ifdef PROJ_GPS2
    gps_data gps2_tmp;
  #endif

  #ifdef DEBUG_MODE
    Serial.println(F("Started reading EEPROM Log."));
    Serial.print(F("DPTR val: ")); Serial.println(eeprom.dptr);Serial.println();
  #endif
  
  for(uint16_t address = DATA_START_ADD; address < eeprom.dptr; ){
    #ifdef DEBUG_MODE
      Serial.print(F("address: ")); Serial.println(address);Serial.println();
    #endif
    #ifdef PROJ_BMP
      pres = *((uint32_t*)eeprom.read(address, sizeof(uint32_t)));
      address += sizeof(uint32_t);
      temp = *((uint16_t*)eeprom.read(address, sizeof(uint16_t)));
      address += sizeof(uint16_t);
      
      Serial.println(F("BMP data:"));
      Serial.print(F("Raw pressure: "));Serial.print(pres,HEX);Serial.print(F("\t\t Raw temp: "));Serial.println(temp,HEX);
      Serial.print(F("Converted pressure: "));Serial.print(slave_packet.convertPressure());Serial.print(F("\t Converted temp: "));Serial.println(slave_packet.convertTemperature());
      Serial.print(F("Sea Level Pressure: "));Serial.print(slave_packet.convertSealevelPressure());Serial.print(F("\t Altitude: "));Serial.println(slave_packet.convertAltitude());
      Serial.println();
    #endif
    #ifdef PROJ_GPS1
      gps1_tmp = *((gps_data*)eeprom.read(address, sizeof(gps_data)));
      address += sizeof(gps_data);

      Serial.println(F("GPS 1 data:"));
      Serial.print(F("Date: "));Serial.print(gps1_tmp.date);Serial.print(F("\t Time: "));Serial.println(gps1_tmp.Time);
      Serial.print(F("Latitude: "));Serial.print(gps1_tmp.lat);Serial.print(F("\t Longitude: "));Serial.print(gps1_tmp.lon);Serial.print(F("\t Altitude: "));Serial.print(gps1_tmp.alt);Serial.print(F("\t Speed: "));Serial.println(gps1_tmp.spd);
      Serial.print(F("Satellites: "));Serial.print(gps1_tmp.numSat);Serial.print(F("\t Hdop: "));Serial.print(gps1_tmp.Hdop);Serial.print(F("\t Info age: "));Serial.println(gps1_tmp.age);
      Serial.println();
    #endif
    #ifdef PROJ_GPS2
      gps2_tmp = *((gps_data*)eeprom.read(address, sizeof(gps_data)));
      address += sizeof(gps_data);
      
      Serial.println(F("GPS 2 data:"));
      Serial.print(F("Date: "));Serial.print(gps2_tmp.date);Serial.print(F("\t Time: "));Serial.println(gps2_tmp.Time);
      Serial.print(F("Latitude: "));Serial.print(gps2_tmp.lat);Serial.print(F("\t Longitude: "));Serial.print(gps2_tmp.lon);Serial.print(F("\t Altitude: "));Serial.print(gps2_tmp.alt);Serial.print(F("\t Speed: "));Serial.println(gps2_tmp.spd);
      Serial.print(F("Satellites: "));Serial.print(gps2_tmp.numSat);Serial.print(F("\t Hdop: "));Serial.print(gps2_tmp.Hdop);Serial.print(F("\t Info age: "));Serial.println(gps2_tmp.age);
      Serial.println();
    #endif
  } 
}

/*******************************************************************************
                                  SD LOG
*******************************************************************************/
#ifdef PROJ_SD
void writeLogSD(){
  #ifdef DEBUG_SD
  Serial.println("Writing SD log.");
  #endif
  sdFile = SD.open(SD_FILE_NAME, FILE_WRITE);
  if(sdFile){
    #ifdef DEBUG_SD
    Serial.print("File \"");Serial.print(SD_FILE_NAME);Serial.println("\" opened successfully");
    #endif
    #ifdef PROJ_BMP
      sdFile.println(F("BMP data:"));
      sdFile.print(F("Raw pressure: "));sdFile.print(slave_packet.BMP_pressure,HEX);sdFile.print(F("\t\t Raw temp: "));sdFile.println(slave_packet.BMP_temperature,HEX);
      sdFile.print(F("Converted pressure: "));sdFile.print(slave_packet.convertPressure());sdFile.print(F("\t Converted temp: "));sdFile.println(slave_packet.convertTemperature());
      sdFile.print(F("Sea Level Pressure: "));sdFile.print(slave_packet.convertSealevelPressure());sdFile.print(F("\t Altitude: "));sdFile.println(slave_packet.convertAltitude());
      sdFile.println();
      #ifdef DEBUG_SD
      Serial.println("BMP data written to SD");
      #endif
    #endif
    #ifdef PROJ_GPS1
      sdFile.println(F("GPS 1 data:"));
      sdFile.print(F("Date: "));sdFile.print(slave_packet.gps1.date);sdFile.print(F("\t Time: "));sdFile.println(slave_packet.gps1.Time);
      sdFile.print(F("Latitude: "));sdFile.print(slave_packet.gps1.lat);sdFile.print(F("\t Longitude: "));sdFile.print(slave_packet.gps1.lon);sdFile.print(F("\t Altitude: "));sdFile.print(slave_packet.gps1.alt);sdFile.print(F("\t Speed: "));sdFile.println(slave_packet.gps1.spd);
      sdFile.print(F("Satellites: "));sdFile.print(slave_packet.gps1.numSat);sdFile.print(F("\t Hdop: "));sdFile.print(slave_packet.gps1.Hdop);sdFile.print(F("\t Info age: "));sdFile.println(slave_packet.gps1.age);
      sdFile.println();
      #ifdef DEBUG_SD
      Serial.println("GPS1 data written to SD");
      #endif
    #endif
    #ifdef PROJ_GPS2
      sdFile.println(F("GPS 2 data:"));
      sdFile.print(F("Date: "));sdFile.print(slave_packet.gps2.date);sdFile.print(F("\t Time: "));sdFile.println(slave_packet.gps2.Time);
      sdFile.print(F("Latitude: "));sdFile.print(slave_packet.gps2.lat);sdFile.print(F("\t Longitude: "));sdFile.print(slave_packet.gps2.lon);sdFile.print(F("\t Altitude: "));sdFile.print(slave_packet.gps2.alt);sdFile.print(F("\t Speed: "));sdFile.println(slave_packet.gps2.spd);
      sdFile.print(F("Satellites: "));sdFile.print(slave_packet.gps2.numSat);sdFile.print(F("\t Hdop: "));sdFile.print(slave_packet.gps2.Hdop);sdFile.print(F("\t Info age: "));sdFile.println(slave_packet.gps2.age);
      sdFile.println();
      #ifdef DEBUG_SD
      Serial.println("GPS2 data written to SD");
      #endif
    #endif
  }
  sdFile.close();
  #ifdef DEBUG_SD
    Serial.println("Finished writing data to SD card");Serial.println();
  #endif
}
#endif

/*******************************************************************************
                             LORA COMMUNICATION
*******************************************************************************/
void sendLoRa(){
  #ifdef DEBUG_LORA
    Serial.println("Sending LoRa.");
  #endif
  LoRa.beginPacket();
  #ifdef PROJ_BMP
    LoRa.print(slave_packet.BMP_pressure);LoRa.print("\n");
    LoRa.print(slave_packet.BMP_temperature);LoRa.print("\n");
    #ifdef DEBUG_LORA
      Serial.println("BMP data sended.");
    #endif
  #endif
  #ifdef PROJ_GPS1
    LoRa.print(slave_packet.gps1.date);LoRa.print("\n");
    LoRa.print(slave_packet.gps1.Time);LoRa.print("\n");
    LoRa.print(slave_packet.gps1.lat);LoRa.print("\n");
    LoRa.print(slave_packet.gps1.lon);LoRa.print("\n");
    LoRa.print(slave_packet.gps1.alt);LoRa.print("\n");
    LoRa.print(slave_packet.gps1.spd);LoRa.print("\n");
    LoRa.print(slave_packet.gps1.numSat);LoRa.print("\n");
    LoRa.print(slave_packet.gps1.Hdop);LoRa.print("\n");
    LoRa.print(slave_packet.gps1.age);LoRa.print("\n");
    #ifdef DEBUG_LORA
      Serial.println("GPS data sended.");
    #endif
  #endif
  LoRa.endPacket();
  #ifdef DEBUG_LORA
    Serial.println("Finished sending LoRa packet.");
    Serial.println();
  #endif
}

/*******************************************************************************
                             AUXILIAR FUNCTIONS
*******************************************************************************/
int32_t i2c_read(int num_bytes){
  byte i2c_recieved;
  int32_t ret = 0;
  uint8_t i = 0;
  #ifdef DEBUG_MODE_I2C
    Serial.print(F("Reading "));Serial.print(num_bytes, DEC);Serial.print(F(" bytes from slave:"));
  #endif
  while(Wire.available() && i < num_bytes){
    i2c_recieved = Wire.read();
    ret = ret << 8;
    ret += i2c_recieved;
    i++;
    #ifdef DEBUG_MODE_I2C
      Serial.print(i2c_recieved,BIN);Serial.print(F(" "));
    #endif
  }
  #ifdef DEBUG_MODE_I2C
    Serial.println();
    Serial.print(F("Final value: "));Serial.print(ret,DEC);
    Serial.print(F(" Binary value: "));Serial.println(ret,BIN);
  #endif
  return ret;
}

int32_t i2c_request_and_read(int num_bytes){
  byte i2c_recieved;
  int32_t ret = 0;
  uint8_t i = 0;
  #ifdef DEBUG_MODE_I2C
    Serial.print(F("Requesting "));Serial.print(num_bytes, DEC);Serial.println(F(" bytes from slave:"));
    Serial.println(F("Bytes recieved: "));
  #endif
  Wire.requestFrom(SLAVE_I2C_ADD, num_bytes);    // request "num_bytes" bytes from slave (device #8)
  while(Wire.available() && i < num_bytes){
    i2c_recieved = Wire.read();
    ret = ret << 8;
    ret += i2c_recieved;
    i++;
    #ifdef DEBUG_MODE_I2C
      Serial.print(i2c_recieved,BIN);Serial.print(F(" "));
    #endif
  }
  #ifdef DEBUG_MODE_I2C
    Serial.println(F(" "));
    Serial.print(F("Final value: "));Serial.println(ret,DEC);
    Serial.print(F("Binary form: "));Serial.println(ret,BIN);
  #endif
  return ret;
}

int32_t i2c_read32(){
  byte i2c_recieved;
  int32_t ret = 0;
  uint8_t i = 0;
  #ifdef DEBUG_MODE
    Serial.println(F("Requesting 4bytes (32 bits) from slave:"));
    Serial.println(F("Bytes recieved: "));
  #endif
  Wire.requestFrom(SLAVE_I2C_ADD, 4);    // request 4 bytes from slave (device #8)
  while(Wire.available() && i < 4){
    i2c_recieved = Wire.read();
    ret = ret << 8;
    ret += i2c_recieved;
    i++;
    #ifdef DEBUG_MODE
      Serial.print(i2c_recieved,BIN);Serial.print(F(" "));
    #endif
  }
  #ifdef DEBUG_MODE
    Serial.println(F(" "));
    Serial.print(F("Final value: "));Serial.println(ret,DEC);
    Serial.print(F("Binary form: "));Serial.println(ret,BIN);
  #endif
  return ret;
}

void setSlaveTS(byte transmissionState){
  Wire.beginTransmission(SLAVE_I2C_ADD);
  Wire.write(transmissionState);
  Wire.endTransmission();
}
