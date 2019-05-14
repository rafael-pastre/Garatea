/*
 * TO-DO List:
 * check if latitude and longitude can be written with negative values therefore NS and EW can be unused
 */
/*******************************************************************************
                                INCLUDES & DEFINES
*******************************************************************************/
// ARDUINO INCLUDES
#include <zenith.h>                               // main library

// TRASMISSION STATE IDs
// transmissionState defines what information the slave will send to master
// must set transmissionState before every read call on the slave
#define BMP_PRES_TSID   0                         // transmissionState value for sending BMP pression to master
#define BMP_TEMP_TSID   1                         // transmissionState value for sending BMP temperature to master
#define BMP_ALT_TSID    2                         // transmissionState value for sending BMP altitude to master
#define GPS_TIME_TSID   3                         // transmissionState value for sending GPS time to master
#define GPS_LAT_TSID    4                         // transmissionState value for sending GPS latitude to master
#define GPS_NS_TSID     5                         // transmissionState value for sending GPS North/South to master
#define GPS_LONG_TSID   6                         // transmissionState value for sending GPS longitude to master
#define GPS_EW_TSID     7                         // transmissionState value for sending GPS East/West to master
#define GPS_QUAL_TSID   8                         // transmissionState value for sending GPS quality to master
#define GPS_SAT_TSID    9                         // transmissionState value for sending GPS satellites number to master
#define GPS_HDOP_TSID   10                        // transmissionState value for sending GPS horizontal precision to master
#define GPS_AGE_TSID    11                        // transmissionState value for sending GPS information age to master


/*******************************************************************************
                                GLOBAL VARIABLES
*******************************************************************************/
// MASTER-SLAVE COMMUNICATION
byte transmissionState=0;                         // variable that controls which data the slave will send to master via I2C bus

/*******************************************************************************
                                    SLAVE
*******************************************************************************/
void setup() {                         

  #ifdef DEBUG_MODE
  Serial.begin(DEBUG_BAUD);
  #endif
  
  // I2C SETUP
  Wire.begin(SLAVE_I2C_ADD);                      // join I2C bus with address defined in SLAVE_I2C_ADD
  Wire.onRequest(requestEvent);                   // defines the event for requesting data in I2C bus
  Wire.onReceive(receiveEvent);                   // defines the event for recieving data in I2C bus

  // BMP INITIALIZATION
  #ifdef PROJ_BMP
  BMP_init();
  #endif
  // GPS INITIALIZATION
  #ifdef PROJ_GPS
  GPS_init();
  #endif
}

void loop() {
  BMP_read();

  BMP_print_Info();
  
  if(GPS_update() == NO_ERR){
    //Serial.print("I:");
    //printString(GPSBuffer, GPSBufferLength);
    //Serial.print(":F");
    GPS_Process_NMEA_Line();
  }

}

/*******************************************************************************
                              MASTER-SLAVE EVENTS
*******************************************************************************/
void requestEvent() {
  // function that executes whenever data is requested by master
  // this function is registered as an event, see setup()
  switch(transmissionState){
    case BMP_PRES_TSID:
      i2c_send(BMP_pressure);
    break;
    case BMP_TEMP_TSID:
      i2c_send(BMP_temperature);
    break;
    case BMP_ALT_TSID:
      i2c_send(BMP_Altitude);
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
