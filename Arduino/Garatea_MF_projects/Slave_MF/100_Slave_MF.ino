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
  #ifdef DEBUG_MODE
    Serial.begin(DEBUG_BAUD);
    Serial.println("Debug Mode");
  #endif
  
  // I2C SETUP
  Wire.begin(SLAVE_I2C_ADD);                      // join I2C bus with address defined in SLAVE_I2C_ADD
  Wire.onRequest(requestEvent);                   // defines the event for requesting data in I2C bus
  Wire.onReceive(receiveEvent);                   // defines the event for recieving data in I2C bus

  // BMP INITIALIZATION
  #ifdef PROJ_BMP
    bmp.init();
  #endif
  // GPS INITIALIZATION
  #ifdef PROJ_GPS1
    gps1.init();
    gps1.setGPS_DynamicMode6();
  #endif
  #ifdef PROJ_GPS2
    gps1.init();
  #endif
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
    delay(DEBUG_DELAY);
  #endif
}

/*******************************************************************************
                              MASTER-SLAVE EVENTS
*******************************************************************************/
void requestEvent() {
  // function that executes whenever data is requested by master
  // this function is registered as an event, see setup()

  #ifdef DEBUG_MODE
    Serial.print("Trasnmission state: ");Serial.println(transmissionState);
  #endif
  
  switch(transmissionState){
    #ifdef PROJ_BMP
    case BMP_PRES_TSID:
      i2c_send(bmp.raw_pressure(), 4);
    break;
    case BMP_TEMP_TSID:
      i2c_send(bmp.raw_temperature(), 2);
    break;
    case BMP_CAL_TSID:
      i2c_send(bmp.get_ac1(), 2);
      i2c_send(bmp.get_ac2(), 2);
      i2c_send(bmp.get_ac3(), 2);
      i2c_send(bmp.get_ac4(), 2);
      i2c_send(bmp.get_ac5(), 2);
      i2c_send(bmp.get_ac6(), 2);
      i2c_send(bmp.get_b1(), 2);
      i2c_send(bmp.get_b2(), 2);
      i2c_send(bmp.get_mb(), 2);
      i2c_send(bmp.get_mc(), 2);
      i2c_send(bmp.get_md(), 2);
    break;
    #endif    
  }
}

void receiveEvent(int howMany) {
  // function that executes whenever data is received from master
  // this function is registered as an event, see setup()
  while(Wire.available()) {
    transmissionState = Wire.read();
  }
}

void i2c_send(int32_t v, int num_bytes){
  // function for sending a 32-bit integer in the I2C bus
  // only a 8-bit char can be send in the I2C bus at a time
  // the integer bytes are sended from the msb to the lsb
  byte i2c_send;

  #ifdef DEBUG_MODE
    Serial.print("Sending ");Serial.print(num_bytes, DEC);Serial.print(" bytes value: ");Serial.println(v, DEC);
    Serial.print("Binary form value: ");Serial.println(v, BIN);
    Serial.print("Bytes sended: ");
  #endif
  
  swap_bytes(&v, num_bytes);
  for(int i = 0; i < num_bytes; i++){
    i2c_send = v;
    Wire.write(i2c_send);
    v = v >> 8;
    #ifdef DEBUG_MODE
      Serial.print(i2c_send, BIN);Serial.print(" ");
    #endif
  }
  #ifdef DEBUG_MODE
    Serial.println(" ");Serial.println(" ");
  #endif
}

/*******************************************************************************
                       MASTER-SLAVE AUXILIAr FUNCTIONS
*******************************************************************************/
void i2c_send32(int32_t v){
  // function for sending a 32-bit integer in the I2C bus
  // only a 8-bit char can be send in the I2C bus at a time
  // the integer bytes are sended from the msb to the lsb
  byte i2c_send;

  #ifdef DEBUG_MODE
    Serial.print("Sending 32 bits value: ");Serial.println(v, DEC);
    Serial.print("Binary form value: ");Serial.println(v, BIN);
    Serial.print("Bytes sended: ");
  #endif
  
  v = swap_bytes(v);
  for(int i = 0; i < 4; i++){
    i2c_send = v;
    Wire.write(i2c_send);
    v = v >> 8;
    #ifdef DEBUG_MODE
      Serial.print(i2c_send, BIN);Serial.print(" ");
    #endif
  }
  #ifdef DEBUG_MODE
    Serial.println(" ");Serial.println(" ");
  #endif
}
