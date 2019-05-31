#include <string.h>
#include <SoftwareSerial.h>

#define GPS_TX          10
#define GPS_RX          11
#define GPS_ENABLE      12
#define GPS_BAUD_RATE   9600
#define GPS_BUFF_LEN    82
#define NMEA_DIV_LEN    20
#define NMEA_SUB_LEN    10

#define NO_ERR          0
#define INDEX_ERR       1
#define READ_ERR        2

SoftwareSerial gpsSerial(GPS_TX, GPS_RX);         // defines serial(UART) communication with GPS

byte    GPSBuffer[GPS_BUFF_LEN];                  // buffer for NMEA comunication
int32_t GPSBufferLength=0;                        // stores the length of the GPSBuffer after 'checkGPS'
int32_t NMEADiv[NMEA_DIV_LEN];                    // position of the GPSBuffer terms
int32_t NMEADivLen=0;                             // curent number of terms(divisions) in the GPSBuffer. limited by NMEA_SUB_LEN
int32_t NMEASub[NMEA_SUB_LEN];                    // position of a term subdivisions
int32_t NMEASubLen=0;                             // curent number of terms(divisions) in the GPSBuffer. limited by NMEA_SUB_LEN
char    NMEATalkerID[3];                          // sender of NMEA message
char    NMEAMnemonic[4];                          // mnemonic for message type
int32_t GPS_hour=0, GPS_min=0, GPS_sec=0;         // time
int32_t GPS_lat_int=0, GPS_lat_frac=0;            // latitude
char    GPS_NS;                                   // north/south
int32_t GPS_long_int=0, GPS_long_frac=0;          // longitude
char    GPS_EW;                                   // east/west
char    GPS_quality='0';                          // quality
int32_t GPS_Satellites=0;                         // number of satelites
int32_t GPS_HDOP_int=0, GPS_HDOP_frac=0;          // horizontal precision
int32_t GPS_Altitude=0;                           // altitude
int32_t GPS_age_int=0, GPS_age_frac=0;            // information age
 
void setup()
{
  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, HIGH);
  gpsSerial.begin(GPS_BAUD_RATE);
  Serial.begin(115200);
  Serial.println("Test Program");
}
 
void loop()
{
  CheckGPS();
  //GPS_print_Info();
}

void printString(byte* str, int len) {
  for(int i = 0; i < len; i++)
    Serial.write(str[i]);
}

void CheckGPS()
{
  int inByte;
  while (gpsSerial.available() > 0){
    inByte = gpsSerial.read();
    Serial.write(inByte);                         // Output raw NMEA string from GPS. Use for debugging

    if ((inByte =='$') || (GPSBufferLength >= 80))
      GPSBufferLength = 0;
    if (inByte != '\r')
      GPSBuffer[GPSBufferLength++] = inByte;
    if (inByte == '\n'){
      printString(GPSBuffer, GPSBufferLength);
      ProcessGPSLine();
      GPSBufferLength = 0;
    }
  }
  //Serial.println("GPS unnavaliable");
}

void GPS_print_Info(){
    Serial.print("Altitude: ");
    Serial.println(GPS_Altitude);
    
    Serial.print("Satellites: ");
    Serial.println(GPS_Satellites);
    
    Serial.print("Horario: ");
    Serial.print(GPS_hour);
    Serial.print(":");
    Serial.print(GPS_min);
    Serial.print(":");
    Serial.println(GPS_sec);
    
    Serial.print("Latitude: ");
    Serial.print(GPS_lat_int);
    Serial.print(",");
    Serial.print(GPS_lat_frac);
    Serial.println(GPS_NS);

    Serial.print("Longitude: ");
    Serial.print(GPS_long_int);
    Serial.print(",");
    Serial.print(GPS_long_frac);
    Serial.println(GPS_EW);
    
    Serial.print("Qualidade: ");
    Serial.println(GPS_quality);

    Serial.print("Precisao: ");
    Serial.print(GPS_HDOP_int);
    Serial.print(",");
    Serial.println(GPS_HDOP_frac);

    Serial.print("Idade da informacao: ");
    Serial.print(GPS_age_int);
    Serial.print(",");
    Serial.println(GPS_age_frac);
}

void ProcessGPSLine(){
  Serial.println("Processing Line");
  // get NMEA talker ID
  //  ______________________
  // |    GNSS     |TalkerID|
  // |-------------|--------|
  // |GPS,SBAS,QZSS|   GP   |
  // |   GLONASS   |   GL   |
  // |   Galileo   |   GA   |
  // |   BeiDou    |   GB   |
  // |    any      |   GN   |
  //  ----------------------
  readStringBuffer(NMEAMnemonic, 0, 2);
  NMEATalkerID[0] = GPSBuffer[1];
  NMEATalkerID[1] = GPSBuffer[2];
  NMEATalkerID[2] = '\0';

  // get NMEA Mnemonic
  readStringBuffer(NMEAMnemonic, 3, 3);
  /**
  NMEAMnemonic[0] = GPSBuffer[3];
  NMEAMnemonic[1] = GPSBuffer[4];
  NMEAMnemonic[2] = '\0';
  */
  // process NMEA Mnemonic (implementar outras mensagens)
  Serial.println(NMEAMnemonic);
  if(!strcmp(NMEAMnemonic,"GGA"))
    ProcessGNGGACommand();
  /**
  if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'N') && (GPSBuffer[3] == 'G') && (GPSBuffer[4] == 'G') && (GPSBuffer[5] == 'A')){
    Serial.println("GNGGA Detected!");
    ProcessGNGGACommand();
    GPS_print_Info();
  }
  */
}

int32_t readStringBuffer(char* str, int32_t st_index, int32_t N){
  if(N < 0)
    return INDEX_ERR;

  for(int i = 0; i < N; i++){
    str[i] = GPSBuffer[st_index+i];
  }

  str[N] = '\0';
  return NO_ERR;
}
int32_t readBufferIndex(int32_t st_index, int32_t end_index, char ignore, int32_t* val){
  if(st_index > end_index)
    return INDEX_ERR;
    
  *val = 0;
  
  while(st_index <= end_index){
    if(GPSBuffer[st_index] != ignore){
      st_index++;
      continue;
    }
    if ((GPSBuffer[st_index] < '0') || (GPSBuffer[st_index] > '9'))
      return READ_ERR;

    *val = (*val)*10;
    *val += GPSBuffer[st_index];
    st_index++;
  }
  return NO_ERR;
}

void ProcessGNGGACommand(){
  Serial.println("GGA");
  // Format:  $xxGGA,time,lat,NS,long,EW,quality,numSat,HDOP,alt,M,sep,M,diffAge,diff,Station*cs<CR><LF>
  // Example: $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  
  int32_t i, j, k, subdivision;
  int32_t Altitude;
  
  GPS_hour = 0; GPS_min = 0; GPS_sec = 0; // case 1: time
  GPS_lat_int=0; GPS_lat_frac=0;          // case 2: lat
  GPS_long_int=0; GPS_long_frac=0;        // case 4: long
  GPS_Satellites = 0;                     // case 7: numSat
  GPS_HDOP_int=0; GPS_HDOP_frac=0;        // case 8: precisao horizontal (HDOP)
  Altitude = 0;                           // case 9: altitude
  GPS_age_int=0; GPS_age_int=0;           // case 13: idade da informacao
 
  for (i=7, j=1, k=0, subdivision = 1; (i<GPSBufferLength) && (j<16); i++, k++) // We start at 7 so we ignore the '$GNGGA,'
  {
    if (GPSBuffer[i] == ',')
    {
      j++;    // Segment index
      k=0;    // Index into target variable
      subdivision = 1;
    }
    else
    {
      switch(j){
        case 1: // time
          if(k == 2){
            subdivision++;
            k = 0;
          }
          switch(subdivision){
            case 1: // horas
            if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
              GPS_hour = GPS_hour*10;
              GPS_hour += (int32_t)(GPSBuffer[i] - '0');
            }
            break;
            case 2: // min
            if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
              GPS_min = GPS_min*10;
              GPS_min += (int32_t)(GPSBuffer[i] - '0');
            }
            break;
            case 3: // seg
            if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
              GPS_sec = GPS_sec*10;
              GPS_sec += (int32_t)(GPSBuffer[i] - '0');
            }
            break;
          }
        break;
        case 2: // lat
          switch(subdivision){
            case 1: // parte inteira
              if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
                GPS_lat_int = GPS_lat_int*10;
                GPS_lat_int += (int32_t)(GPSBuffer[i] - '0');
              }
              if(k == 2){
                subdivision++;
                k = 0;
              }
            break;
            case 2: // parte fracionaria
              if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9') && (GPSBuffer[i] != '.')){
                GPS_lat_frac = GPS_lat_frac*10;
                GPS_lat_frac += (int32_t)(GPSBuffer[i] - '0');
              }
            break;
          }
        break;
        case 3: // N/S
          if ((GPSBuffer[i] == 'N') || (GPSBuffer[i] == 'S')){
              GPS_NS = GPSBuffer[i];
          }
        break;
        case 4: // lon
          switch(subdivision){
            case 1: // parte inteira
              if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
                GPS_long_int = GPS_long_int*10;
                GPS_long_int += (int32_t)(GPSBuffer[i] - '0');
              }
              if(k == 3){
                subdivision++;
                k = 0;
              }
            break;
            case 2: // parte fracionaria
              if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9') && (GPSBuffer[i] != '.')){
                GPS_long_frac = GPS_long_frac*10;
                GPS_long_frac += (int32_t)(GPSBuffer[i] - '0');
              }
            break;
          }
        break;
        case 5: // E/W
          if ((GPSBuffer[i] == 'E') || (GPSBuffer[i] == 'W')){
              GPS_EW = GPSBuffer[i];
          }
        break;
        case 6: // quality
          if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
            GPS_quality = GPSBuffer[i];
          }
        break;
        case 7: // numSat
          // Satellite Count
          if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9'))
          {
            GPS_Satellites = GPS_Satellites * 10;
            GPS_Satellites += (int32_t)(GPSBuffer[i] - '0');
          } 
        break;
        case 8: // precisao(HDOP)
          switch(subdivision){
            case 1: // horas
            if((GPSBuffer[i] == '.')){
              subdivision++;
              k = 0;
            }
            else if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
              GPS_HDOP_int = GPS_HDOP_int*10;
              GPS_HDOP_int += (int32_t)(GPSBuffer[i] - '0');
            }
            break;
            case 2: // min
            if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
              GPS_HDOP_frac = GPS_HDOP_frac*10;
              GPS_HDOP_frac += (int32_t)(GPSBuffer[i] - '0');
            }
            break;
          }
        break;
        case 9: // altitude
          if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9') && subdivision)
          {
            Altitude = Altitude * 10;
            Altitude += (int32_t)(GPSBuffer[i] - '0');
          }
          else
          {
            subdivision = 0;
          }
        break;
        case 10: // alt Unit
        break;
        case 11: // 
        break;
        case 12: // 
        break;
        case 13: // idade Informacao
          switch(subdivision){
            case 1: // horas
            if((GPSBuffer[i] == '.')){
              subdivision++;
              k = 0;
            }
            if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
              GPS_age_int = GPS_HDOP_int*10;
              GPS_age_int += (int32_t)(GPSBuffer[i] - '0');
            }
            break;
            case 2: // min
            if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
              GPS_age_frac = GPS_HDOP_frac*10;
              GPS_age_frac += (int32_t)(GPSBuffer[i] - '0');
            }
            break;
          }
        break;
        case 14: // id station
        break;
        case 15: // checksum
        break;
      }
    }
    GPS_Altitude = Altitude;
  }
}
/**
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define GPS_ENABLE 12

SoftwareSerial gpsSerial(10, 11); // RX, TX
TinyGPS gps1; //Criando o GPS

long latitude, longitude;
unsigned long idadeInfo;
int ano;
byte mes, dia, hora, minuto, segundo, centesimo;
float altitudeGPS;
bool recebido;
char cIn;
float velocidade;
unsigned short satelites;
unsigned long precisao;
unsigned long sentido;
 
void setup() {
 gpsSerial.begin(9600); //valeocidade do GPS, não é possivel modificar
 Serial.begin(9600); // Velocidade do serial
 pinMode(GPS_ENABLE, OUTPUT);
 digitalWrite(GPS_ENABLE, HIGH);
 
Serial.println("Aguardando a concecção com os satelites");
}
 
void loop() {
while (gpsSerial.available()) { //realiza diversas leituras até conseguir se conectar
 recebido = false; // indicará quando o GPS conseguiu se conectar aos satélites
 //Serial.println("a");
 cIn = gpsSerial.read();
 //Serial.print(cIn, DEC);
 //Serial.print(" ");
 recebido = gps1.encode(cIn);
 //Serial.println(recebido);
 
 //}
 
// quando conseguir se conectar aos satélites ele irá realizar as leituras
 //if (recebido) {
 Serial.println("----------------------------------------");
 
 //Latitude e Longitude
 gps1.get_position(&latitude, &longitude, &idadeInfo); // obtem a latitude, longitude e a idade da informação
 
// se a latitude for algo valido ela será impressa
 if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
 Serial.print("Latitude: ");
 Serial.println(float(latitude) / 100000, 6);
 }
 
// se a longitude for algo valido ela será impressa
 if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
 Serial.print("Longitude: ");
 Serial.println(float(longitude) / 100000, 6);
 }
 
 // se a data da informação for algo valido ela será impressa
 if (idadeInfo != TinyGPS::GPS_INVALID_AGE) {
 Serial.print("Idade da Informacao (ms): ");
 Serial.println(idadeInfo);
 }
 
 
 //Obtem o Dia e Hora no GMT, ou seja, há 3 horas de diferença com o horário do Brasil
 gps1.crack_datetime(&ano, &mes, &dia, &hora, &minuto, &segundo, &centesimo, &idadeInfo); //obtendo a data, horário e a idade da informação
 
 // imprimindo os dados
 Serial.print("Data (GMT): ");
 Serial.print(dia);
 Serial.print("/");
 Serial.print(mes);
 Serial.print("/");
 Serial.println(ano);
 
Serial.print("Horario (GMT): ");
 Serial.print(hora);
 Serial.print(":");
 Serial.print(minuto);
 Serial.print(":");
 Serial.print(segundo);
 Serial.print(":");
 Serial.println(centesimo);
 
 
 //Obtendo a altitude
 altitudeGPS = gps1.f_altitude();
 
// se a altitude for algo valido e diferente de 1000000Cm ela será impressa
 if ((altitudeGPS != TinyGPS::GPS_INVALID_ALTITUDE) && (altitudeGPS != 1000000)) {
 Serial.print("Altitude (cm): ");
 Serial.println(altitudeGPS);
 }
 
 
 //obtem a velocidade de locomoção do gps em km/h
 velocidade = gps1.f_speed_kmph();
 
Serial.print("Velocidade (km/h): ");
 Serial.println(velocidade, 2); //Conversão de Nós para Km/h
 
//obtem o sentito do movimento em centesima de graus
 sentido = gps1.course();
 
Serial.print("Sentido (grau): ");
 Serial.println(float(sentido) / 100, 2);
 
 
 //obtem a quantidade de satelites e a precisão
 satelites = gps1.satellites();
 precisao = gps1.hdop();
 
if (satelites != TinyGPS::GPS_INVALID_SATELLITES) {
 Serial.print("Satelites: ");
 Serial.println(satelites);
 }
 
if (precisao != TinyGPS::GPS_INVALID_HDOP) {
 Serial.print("Precisao (centesimos de segundo): ");
 Serial.println(precisao);
 }
 }
//}
}
*/
