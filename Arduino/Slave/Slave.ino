#include<zenith.h>
#define PROJ_BMP
#define PROJ_GPS1
/*******************************************************************************
                              GLOBAL VARIABLES
*******************************************************************************/
#define SLAVE_I2C_ADD   8                         // Slave address in I2C bus

// MASTER-SLAVE COMMUNICATION
byte transmissionState=0;                         // variable that controls which data the slave will send to master via I2C bus

#ifdef PROJ_BMP
  zBMP bmp;
#endif
#ifdef PROJ_GPS1
  zGPS gps1(10, 11, 12, 9600, 150);
#endif
#ifdef PROJ_GPS2
  zGPS gps2(3, 4, 9600, 82);
#endif

/*******************************************************************************
                                    SETUP
*******************************************************************************/
void setup() {
  #if defined DEBUG_MODE || defined DEBUG_MODE_HIGH
    Serial.begin(DEBUG_BAUD);
    Serial.println(F("Debug Mode"));
  #endif

  // BMP INITIALIZATION
  #ifdef PROJ_BMP
    while(!bmp.init()){
      #ifdef DEBUG_MODE_HIGH
        Serial.println(F("BMP initialization failed."));
      #endif
    }
    #ifdef DEBUG_MODE_HIGH
      Serial.println(F("BMP initialized."));
      Serial.println(F("Calibration data:"));
      Serial.print(F("ac1: "));Serial.print(bmp.ac1);Serial.print(F("\t hex: "));Serial.println(bmp.ac1,HEX);
      Serial.print(F("ac2: "));Serial.print(bmp.ac2);Serial.print(F("\t hex: "));Serial.println(bmp.ac2,HEX);
      Serial.print(F("ac3: "));Serial.print(bmp.ac3);Serial.print(F("\t hex: "));Serial.println(bmp.ac3,HEX);
      Serial.print(F("ac4: "));Serial.print(bmp.ac4);Serial.print(F("\t hex: "));Serial.println(bmp.ac4,HEX);
      Serial.print(F("ac5: "));Serial.print(bmp.ac5);Serial.print(F("\t hex: "));Serial.println(bmp.ac5,HEX);
      Serial.print(F("ac6: "));Serial.print(bmp.ac6);Serial.print(F("\t hex: "));Serial.println(bmp.ac6,HEX);
      Serial.print(F("b1: "));Serial.print(bmp.b1);Serial.print(F("\t hex: "));Serial.println(bmp.b1,HEX);
      Serial.print(F("b2: "));Serial.print(bmp.b2);Serial.print(F("\t\t hex: "));Serial.println(bmp.b2,HEX);
      Serial.print(F("mb: "));Serial.print(bmp.mb);Serial.print(F("\t hex: "));Serial.println(bmp.mb,HEX);
      Serial.print(F("mc: "));Serial.print(bmp.mc);Serial.print(F("\t hex: "));Serial.println(bmp.mc,HEX);
      Serial.print(F("md: "));Serial.print(bmp.md);Serial.print(F("\t hex: "));Serial.println(bmp.md,HEX);
    #endif
  #endif
  // GPS INITIALIZATION
  #ifdef PROJ_GPS1
    gps1.init();
    gps1.setGPS_DynamicMode6();
  #endif
  #ifdef PROJ_GPS2
    gps1.init();
  #endif

   // I2C SETUP
  Wire.begin(SLAVE_I2C_ADD);                      // join I2C bus with address defined in SLAVE_I2C_ADD
  Wire.onRequest(requestEvent);                   // defines the event for requesting data in I2C bus
  Wire.onReceive(receiveEvent);                   // defines the event for recieving data in I2C bus
}

/*******************************************************************************
                                   LOOP
*******************************************************************************/
void loop() {
  #ifdef PROJ_BMP
    bmp.read();
  #endif
  #ifdef PROJ_GPS1
    gps1.read();
  #endif
  #ifdef PROJ_GPS2
    gps2.read();
  #endif
  #ifdef DEBUG_MODE
    //delay(DEBUG_DELAY);
  #endif
}

/*******************************************************************************
                              MASTER-SLAVE EVENTS
*******************************************************************************/
void requestEvent() {
  // function that executes whenever data is requested by master
  // this function is registered as an event, see setup()

  #ifdef DEBUG_MODE
    Serial.print(F("Trasnmission state: "));Serial.println(transmissionState);
  #endif
  
  switch(transmissionState){
    case BMP_CAL_TSID:
    #ifdef PROJ_BMP
      #ifdef DEBUG_MODE
        Serial.println(F("Sending BMP calibration data:"));
      #endif
      i2c_send(bmp.ac1, 2);
      i2c_send(bmp.ac2, 2);
      i2c_send(bmp.ac3, 2);
      i2c_send(bmp.ac4, 2);
      i2c_send(bmp.ac5, 2);
      i2c_send(bmp.ac6, 2);
      i2c_send(bmp.b1, 2);
      i2c_send(bmp.b2, 2);
      i2c_send(bmp.mb, 2);
      i2c_send(bmp.mc, 2);
      i2c_send(bmp.md, 2);
    #endif
    break;
    case BMP_DATA_TSID:
    #ifdef PROJ_BMP
      #ifdef DEBUG_MODE
        Serial.println(F("Sending BMP data:"));
      #endif
      i2c_send(bmp.pressure, 4);
      i2c_send(bmp.temperature, 2);
    #endif
    break;
    case GPS1_DATA_TSID:
    #ifdef PROJ_GPS1
      #ifdef DEBUG_MODE
        Serial.println(F("Sending GPS1 data:"));
      #endif
      i2c_send(gps1.date, 4);
      i2c_send(gps1.Time, 4);
      i2c_send(gps1.lat, 4);
      i2c_send(gps1.lon, 4);
      i2c_send(gps1.numSat, 4);
      i2c_send(gps1.Hdop, 4);
      i2c_send(gps1.alt, 4);
      i2c_send(gps1.age, 4);
      i2c_send(gps1.spd, 4);
    #endif
    break;
    case GPS2_DATA_TSID:
    #ifdef PROJ_GPS2
      #ifdef DEBUG_MODE
        Serial.println(F("Sending GPS2 data:"));
      #endif
      i2c_send(gps2.date, 4);
      i2c_send(gps2.Time, 4);
      i2c_send(gps2.lat, 4);
      i2c_send(gps2.lon, 4);
      i2c_send(gps2.numSat, 4);
      i2c_send(gps2.Hdop, 4);
      i2c_send(gps2.alt, 4);
      i2c_send(gps2.age, 4);
      i2c_send(gps2.spd, 4);
    #endif
    break;
  }
}

void receiveEvent(int howMany) {
  // function that executes whenever data is received from master
  // this function is registered as an event, see setup()
  while(Wire.available()) {
    transmissionState = Wire.read();
  }
}

/*******************************************************************************
                       MASTER-SLAVE AUXILIAR FUNCTIONS
*******************************************************************************/
void i2c_send(int32_t v, int num_bytes){
  // function for sending a 32-bit integer in the I2C bus
  // only a 8-bit char can be send in the I2C bus at a time
  // the integer bytes are sended from the msb to the lsb
  byte i2c_send;

  #ifdef DEBUG_MODE
    Serial.print(F("Sending "));Serial.print(num_bytes, DEC);Serial.print(F(" bytes value: "));Serial.println(v, DEC);
    Serial.print(F("Binary form value: "));Serial.println(v, BIN);
    Serial.print(F("Bytes sended: "));
  #endif
  
  swap_bytes(&v, num_bytes);
  for(int i = 0; i < num_bytes; i++){
    i2c_send = v;
    Wire.write(i2c_send);
    v = v >> 8;
    #ifdef DEBUG_MODE
      Serial.print(i2c_send, BIN);Serial.print(F(" "));
    #endif
  }
  #ifdef DEBUG_MODE
    Serial.println(F(" "));Serial.println(F(" "));
  #endif
}

void i2c_send32(int32_t v){
  // function for sending a 32-bit integer in the I2C bus
  // only a 8-bit char can be send in the I2C bus at a time
  // the integer bytes are sended from the msb to the lsb
  byte i2c_send;

  #ifdef DEBUG_MODE
    Serial.print(F("Sending 32 bits value: "));Serial.println(v, DEC);
    Serial.print(F("Binary form value: "));Serial.println(v, BIN);
    Serial.print(F("Bytes sended: "));
  #endif
  
  v = swap_bytes(v);
  for(int i = 0; i < 4; i++){
    i2c_send = v;
    Wire.write(i2c_send);
    v = v >> 8;
    #ifdef DEBUG_MODE
      Serial.print(i2c_send, BIN);Serial.print(F(" "));
    #endif
  }
  #ifdef DEBUG_MODE
    Serial.println(F(" "));Serial.println(F(" "));
  #endif
}
