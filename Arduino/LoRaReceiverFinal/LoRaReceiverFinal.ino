#include <SPI.h>
#include <LoRa.h>

#define LORA_CS_PIN 7
#define LORA_RST_PIN -1
#define LORA_DIO0_PIN 2

#define CONSOLE_MODE

void setup() {
  Serial.begin(115200);
  while (!Serial);

  #ifdef CONSOLE_MODE
  Serial.println("LoRa Receiver");
  #endif
  
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  if (!LoRa.begin(433.123E6)) {
    #ifdef CONSOLE_MODE
    Serial.println("Starting LoRa failed!");
    #endif
    while (1);
  }
  
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(11); 
  LoRa.enableCrc();
  LoRa.receive();
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    #ifdef CONSOLE_MODE
    Serial.println("Recieved LoRa Packet:");
    #endif
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    #ifdef CONSOLE_MODE
    Serial.print("RSSI = ");
    #endif
    // print RSSI of packet
    Serial.println(LoRa.packetRssi());
    #ifdef CONSOLE_MODE
    Serial.println();
    #endif
  }
}
