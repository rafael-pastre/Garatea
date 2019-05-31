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

void swap_bytes (void* str, size_t sz){
  // Example: int32_t in  = b3,b2,b1,b0 -> int32_t out = b0,b1,b2,b3
  uint8_t b;
  for(size_t i = 0; i < sz/2; i++){
        b = *((uint8_t*)(str+sz-i-1));
        *((uint8_t*)(str+sz-i-1)) = *((uint8_t*)(str+i));
    *((uint8_t*)(str+i)) = b;
  }
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

int32_t absolute(int32_t x){
  if(x < 0)
    return -x;
  else
    return x;
}
// DEBUG
void printString(byte* str, int len){
  for(int i = 0; i < len; i++)
    Serial.write(str[i]);
}
