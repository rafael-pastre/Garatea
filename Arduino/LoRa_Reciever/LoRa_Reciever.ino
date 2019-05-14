#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define ss 7    //13
#define rst 16  //25
#define dio0 2   //4

int counter = 0;

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  LoRa.setFrequency(433.123E6);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(12E3);
  
  // set LoRa to continuous receive mode
  LoRa.receive();
  
  while (!LoRa.begin(433.123E6)) {
    Serial.println("waiting to receive...");
    delay(5);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0x08);
  Serial.println("LoRa Initializing OK!");   
  
  }

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Receiving packet... \t");

    // read packet
    while (LoRa.available() > 0) {
      if(LoRa.read() > 0){
        Serial.println((char)LoRa.read());
      }
      else{
        Serial.println("Receiving failed!!");
      }
    }
  
    delay(5);
  
  }
}
