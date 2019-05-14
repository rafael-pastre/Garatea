#include "zstdlib.h"

// I2C COMMUNICATION
void i2c_send(int32_t v){
  // function for sending a 32-bit integer in the I2C bus
  // only a 8-bit char can be send in the I2C bus at a time
  // the integer bytes are sended from the msb to the lsb
  byte i2c_send;
  v = swap_bytes(v);
  for(int i = 0; i < 4; i++){
    i2c_send = v;
    Wire.write(i2c_send);
    v = v >> 8;
    // Debug:Serial.print(i2c_send, BIN);
    // Debug:Serial.print(" ");
  }
  // Debug:Serial.println(" ");
}

int32_t i2c_read_int32(){
  byte i2c_recieved;
  int32_t ret = 0;
  
  Wire.requestFrom(SLAVE_I2C_ADD, 4);    // request 4 bytes from slave (device #8)
  while(Wire.available()){
    i2c_recieved = Wire.read();
    ret = ret << 8;
    ret += i2c_recieved;
  }
  return ret;
}

// GENERAL
int32_t swap_bytes (int32_t v){
  // Swaps the order of bytes in a int32_t variable:
  // Example: int32_t in  = b3,b2,b1,b0 -> int32_t out = b0,b1,b2,b3
  int32_t ret = 0;
  byte b;
  for(int i = 0; i < 4; i++){
    ret = ret << 8;
    b = v;
    ret += b;
    v = v >> 8;
  }
  return ret;
}

int32_t findStrIndex(byte* str, int32_t st_index, int32_t end_index, char c){
  // Finds the and returns the index of the first ocurrence of ascii char "c" in "str" string
  // the search begins in "st_index" and ends in "end_index"
  // "st_index" and "end_index" are included in the search
  int32_t i;
  for(i = st_index; i <= end_index; i++)
    if(str[i] == c)
      return i;
  
  return -1;
}

// DEBUG
void printString(byte* str, int len){
  for(int i = 0; i < len; i++)
    Serial.write(str[i]);
}