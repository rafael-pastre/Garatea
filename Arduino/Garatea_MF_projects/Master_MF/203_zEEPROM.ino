zEEPROM::zEEPROM(){
  
}

void zEEPROM::init(){
  mission = read(MISSION_ADD);
    if(mission != MISSION_ID){
      mission = MISSION_ID;
      write(MISSION_ADD, MISSION_ID);
      dptr = DATA_START_ADD;
      write(DPTR_ADD, &dptr, sizeof(uint16_t));
    }
    
    dptr = *((uint16_t*)read(DPTR_ADD, sizeof(uint16_t)));

    #ifdef DEBUG_MODE
      Serial.print(F("Garetea "));Serial.println(mission);
      Serial.print(F("EEPROM loaded with dptr = "));Serial.println(dptr);
    #endif
}

void zEEPROM::write(int16_t w_add, byte data){
  byte i2c_send;
  Wire.beginTransmission(EEPROM_I2C_ADD);
  //Serial.println(w_add, BIN);
  i2c_send = w_add >> 8;
  //Serial.print(i2c_send, BIN);
  //Serial.print(F(" "));
  Wire.write(i2c_send);
  i2c_send = (byte)w_add;
  //Serial.println(i2c_send, BIN);
  Wire.write(i2c_send);
  Wire.write(data);
  Wire.endTransmission();
  delay(8);
}

byte zEEPROM::read(int16_t w_add){
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

void zEEPROM::write(int16_t w_add, void* data, size_t num_bytes){
  int8_t* p = (int8_t*)data;
  for(int i = 0; i < num_bytes; i++)
    write(w_add+i, p[i]);
}

void* zEEPROM::read(int16_t w_add, size_t num_bytes){
  int8_t* p = malloc(sizeof(byte)*num_bytes);
  for(int i = 0; i < num_bytes; i++)
    p[i] = read(w_add + i);
  return (void*)p;
}
