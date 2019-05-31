#ifndef ZEEPROM_H
#define ZEEPROM_H

#define MISSION_ID      96
#define MISSION_ADD     0
#define DPTR_ADD        1
#define DATA_START_ADD  3

#define EEPROM_I2C_ADD 0x50 

class zEEPROM{
  public:
    zEEPROM();
    
    uint16_t dptr;
    uint8_t mission;
    
    void init();
    void write(int16_t w_add, byte data);
    byte read(int16_t w_add);
    void write(int16_t w_add, void* data, size_t num_bytes);
    void* read(int16_t w_add, size_t num_bytes);
  private:

};

#endif//ZEEPROM_H
