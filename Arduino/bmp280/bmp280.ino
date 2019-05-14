// Software para leitura dos sensores BMP280 e NEO-6M
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define PC_Serial_Baud 115200
#define pressao_nivel_mar 1013.25 // Pressao ao nível do mar, é passado como parametro na leitura de altitude, pode ser definido como variável a ser atribuida no início do programa se for mais conveiente
#define GPS_RX 4  // O TX do GPS deve ser ligado neste pino
#define GPS_TX 3  // O RX do GPS deve ser ligado neste pino
#define GPS_Serial_Baud 9600

// Ligacoes do BMP
// VCC e SDO devem ser conectados em 3V3
// GND deve ser conectado com GND
// SDA deve ser conectado na entrada analogica 4
// SCL deve ser conectado na entrada analogica 5

Adafruit_BMP280 sensor_bmp;               // Definicao do Sensor
SoftwareSerial gpsSerial(GPS_RX, GPS_TX); // Definicao da conexao Serial entre o GPS
TinyGPS gps;                              // Definicao do GPS

float temperatura_BMP;        // Temperatura medida pelo BMP, em °C e com precisão 0.01
float pressao_BMP;            // Pressao medida pelo BMP, em Pa e com precisão 0.01
float altitude_BMP;           // Altitude medida pelo BMP, em m e com precisão 0.01

float latitude_GPS;           // Latitude medida pelo GPS, em graus
float longitude_GPS;          // Longitude medida pelo GPS, em graus
unsigned long idadeInfo_GPS;  // Idade da informacao do GPS, em ms
float altitude_GPS;           // Altitude medida pelo GPS, em metros
float velocidade_GPS;         // velocidade medida pelo GPS, em Km/h
unsigned long sentido_GPS;    // sentido medido pelo GPS, verificar unidade
unsigned short satelites_GPS; // Número de satélites que enviaram informações
unsigned long precisao_GPS;   // Precisão das informações


int incomingByte = 0;         // Leitura da transmissao serial com o computador

void readBMP(){
  temperatura_BMP = sensor_bmp.readTemperature();
  pressao_BMP = sensor_bmp.readPressure();
  altitude_BMP = sensor_bmp.readAltitude(pressao_nivel_mar);
}

void readGPS(){
  bool newData = false;
  unsigned long chars;
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;){
    while (gpsSerial.available()){
      char c = gpsSerial.read();
      // Serial.write(c); //apague o comentario para mostrar os dados crus
      if (gps.encode(c)) // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
      }
    }
    if(newData){
      gps.f_get_position(&latitude_GPS, &longitude_GPS, &idadeInfo_GPS);
      altitude_GPS = gps.f_altitude();
      velocidade_GPS = gps.f_speed_kmph();
      sentido_GPS = gps.course();
      satelites_GPS = gps.satellites();
      precisao_GPS = gps.hdop();
    }
}

void printBMP(){
  Serial.print("Temperatura = ");
  Serial.print(temperatura_BMP);
  Serial.println(" *C");
  
  Serial.print("Pressao = ");
  Serial.print(pressao_BMP);
  Serial.println(" Pa");
  
  Serial.print("Altitude aprox BMP = ");
  Serial.print(altitude_BMP); // this should be adjusted to your local forcase
  Serial.println(" m");
  
  Serial.println();
}

void printGPS(){
 if(latitude_GPS != TinyGPS::GPS_INVALID_F_ANGLE){
  Serial.print("Latitude = ");
  Serial.print(latitude_GPS, 6);
  Serial.println(" graus");
 }
 else Serial.println("Latitude = Invalida");

 if(longitude_GPS != TinyGPS::GPS_INVALID_F_ANGLE){
  Serial.print("Longitude = ");
  Serial.print(longitude_GPS, 6);
  Serial.println(" graus");
 }
 else Serial.println("Longitude = Invalida");

 if(idadeInfo_GPS != TinyGPS::GPS_INVALID_AGE){
  Serial.print("Idade da Informacao = ");
  Serial.print(idadeInfo_GPS);
  Serial.println(" ms");
 }
 else Serial.println("Idade da Informacao = Invalida");
 
 if(altitude_GPS != TinyGPS::GPS_INVALID_ALTITUDE){
  Serial.print("Altitude GPS = ");
  Serial.print(altitude_GPS);
  Serial.println(" m");
 }
 else Serial.println("Altitude = Invalida");

 Serial.print("Velocidade = ");
 Serial.print(velocidade_GPS, 2);
 Serial.println(" Km/h");

 Serial.print("Sentido = ");
 Serial.print(sentido_GPS);
 Serial.println(" graus");

 Serial.print("Satelites = ");
 Serial.println(satelites_GPS, 2);

 Serial.print("Precisao = ");
 Serial.println(precisao_GPS);

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(PC_Serial_Baud);

  gpsSerial.begin(GPS_Serial_Baud);
  Serial.println("Sensor GPS encontrado! Digite 'G' para realizar uma leitura");
  
  //Verifica a conexão do sensor BMP280
  if (!sensor_bmp.begin())
    Serial.println("Sensor BMP não encontrado. Verifique as conexoes!");
  else
    Serial.println("Sensor BMP encontrado! Digite 'B' para realizar uma leitura");
}

void loop() {
  // put your main code here, to run repeatedly:
  readBMP(); 
  readGPS();
  if(Serial.available() > 0){           // available() returns the number of bytes available for reading that are stored in the serial buffer
    incomingByte = Serial.read();       // read() returns the next byte in the serial buffer
    if(incomingByte == 'B')
      printBMP();
    if(incomingByte == 'G')
      printGPS();
   }
  delay(200);
}
