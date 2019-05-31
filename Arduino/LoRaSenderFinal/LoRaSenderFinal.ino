#include <SPI.h>
#include <LoRa.h>

#define ss 7    //13
#define rst -1  //25
#define dio0 2   //4

int counter = 0;

void setup() {
  Serial.begin(115200); 
  while (!Serial);

  Serial.println("LoRa Sender");

  LoRa.setPins(ss, rst, dio0);
  
  if (!LoRa.begin(433.123E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20, 1);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(11);
  LoRa.enableCrc();
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.print("\n");
  LoRa.print("LF working");
  LoRa.endPacket();

  counter++;

  delay(1500);
}
