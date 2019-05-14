#include "zBMP.h"

// BMP GLOBAL DATA VARIABLES
Adafruit_BMP085 bmp180;                           // defines the BMP sensor
int32_t BMP_pressure=0;                           // pressure
int32_t BMP_temperature=0;                        // temperature
int32_t BMP_Altitude=0;                           // altitude

// BMP USER FUNCTIONS
err_t BMP_init(){
  if(!bmp180.begin())
    return INIT_ERR;
  else
    return NO_ERR;
}

err_t BMP_read(){
  BMP_read_Pressure(&BMP_pressure);
  BMP_read_Altitude(&BMP_Altitude);
  BMP_read_Temperature(&BMP_temperature);
}

err_t BMP_read_Pressure(int32_t* pressure){
  // Reads pressure from BMP sensor and attributes to "pressure" if the value is valid
  int32_t aux;
  aux = (int32_t)(bmp180.readPressure()); 
  if(aux < 0)
    return INV_DATA_ERR;
  *pressure = aux;
  return NO_ERR;
}

err_t BMP_read_Altitude(int32_t* alt){
  // Reads altitude from BMP sensor and attributes to "alt" if the value is valid
  int32_t aux;
  aux = (int32_t)(100*bmp180.readAltitude());
  if(aux < 0)
    return INV_DATA_ERR;
  *alt = aux;
  return NO_ERR;
}

err_t BMP_read_Temperature(int32_t* temperature){
  // Reads temperature from BMP sensor and attributes to "temperature" if the value is valid
  int32_t aux;
  aux = (int32_t)(100*bmp180.readTemperature());
  if(aux < 0)
    return INV_DATA_ERR;
  *temperature = aux;
  return NO_ERR;
}

// DEBUG
void BMP_print_Pressure(){
  Serial.print("Pressao: ");
  Serial.print(BMP_pressure);  
  Serial.println(" Pa");
}

void BMP_print_Altitude(){
  Serial.print("Altitude: ");
  Serial.print(BMP_Altitude);  
  Serial.println(" m");
}

void BMP_print_Temperature(){
  Serial.print("Temperatura: ");
  Serial.print(BMP_temperature);  
  Serial.println(" C");
}

void BMP_print_Info(){
  BMP_print_Pressure();
  BMP_print_Altitude();
  BMP_print_Temperature();
}