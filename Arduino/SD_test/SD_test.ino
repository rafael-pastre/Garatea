#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN 4
#define SD_FILE_NAME "test2.txt"
File myFile;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Inicializando cartao SD");
  while(!SD.begin(SD_CS_PIN)){
    Serial.println("Falha na inicializacao do cartao SD. Tentando novamente.");
  }
  Serial.println("Cartao SD inicializado");
  myFile = SD.open(SD_FILE_NAME, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open(SD_FILE_NAME);
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
