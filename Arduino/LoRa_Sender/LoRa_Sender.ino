#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define ss 7 // 13
#define rst 16 // 25
#define dio0 2 //4

int counter = 0;

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.begin(433.123E6);
  LoRa.setPins(ss, rst, dio0);
  LoRa.setFrequency(433.123E6);
  
  while (!LoRa.begin(433.123E6)) {
    Serial.println("waiting to send...");
    delay(5);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0x08);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
 
  // send packet
  LoRa.beginPacket();
  LoRa.write('A');
  LoRa.endPacket();
  
  Serial.print("Sending packet A... \t");
  if(LoRa.endPacket()){
    Serial.println("Packet sent succesfully!!");
  }
  else{
    Serial.println("Error senting packet!!");  
  }
  delay(5);
}
