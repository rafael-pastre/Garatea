#include <string.h>
#include <SoftwareSerial.h>

#define GPS_TX          10
#define GPS_RX          11
#define GPS_ENABLE      12
#define GPS_BAUD_RATE   9600
#define GPS_BUFF_LEN    82
#define NMEA_DIV_LEN    20
#define NMEA_SUB_LEN    10

SoftwareSerial gpsSerial(GPS_TX, GPS_RX);         // defines serial(UART) communication with GPS

void setup() {
  // put your setup code here, to run once:
  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, HIGH);
  gpsSerial.begin(GPS_BAUD_RATE);
  Serial.begin(115200);
  delay(2000);
  setGPS_DynamicMode6();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void setGPS_DynamicMode6(){
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };
  
  while(!gps_set_sucess){
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
  Serial.println("GPS set to airborne mode sucesfully");
}

void sendUBX(uint8_t *MSG, uint8_t len) {
  gpsSerial.flush();
  gpsSerial.write(0xFF);
  delay(500);
  Serial.println("Message sent:");
  for(int i=0; i<len; i++){
    gpsSerial.write(MSG[i]);
    Serial.print(MSG[i],HEX);Serial.print(" ");
  }
  Serial.println();
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length lsbyte
  ackPacket[5] = 0x00; // length msbyte
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  Serial.println("Message recieved:");
  while (1) {
    // Test for success
    if (ackByteID > 9){
      Serial.println();
      return true;      // All packets in order!
    }
    
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000){
      Serial.println("Acknowledgement timed out");
      return false;
    }
 
    // Make sure data is available to read
    if (gpsSerial.available()) {
      b = gpsSerial.read();
      Serial.print(b, HEX);Serial.print(" ");
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID])
        ackByteID++;
      else
        ackByteID = 0; // Reset and look again, invalid order
    }
  }
  Serial.println();
}
