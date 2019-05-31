#include "zGPS.h"
// Verificar tamanho da parametros da mensagem NMEA
// Implementar checksum
// Implementar outros tipos de mensagem

// Constructor for GPS without enable pin
zGPS::zGPS(uint8_t tx, uint8_t rx, uint32_t baud, uint16_t buff_len):gpsSerial(tx, rx){
		// pin atribution
		GPS_TX = tx;
		GPS_RX = rx;
		GPS_ENABLE = ALWAYS_ENABLED;
		GPS_BAUD_RATE = baud;
		GPS_BUFF_LEN = buff_len;
		
		//SoftwareSerial::gpsSerial(GPS_TX, GPS_RX);         // defines serial(UART) communication with GPS

}

// Constructor for GPS with enable pin
zGPS::zGPS(uint8_t tx, uint8_t rx, uint8_t en, uint32_t baud, uint16_t buff_len):gpsSerial(tx, rx){
		// pin atribution
		GPS_TX = tx;
		GPS_RX = rx;
		GPS_ENABLE = en;
		GPS_BAUD_RATE = baud;
		GPS_BUFF_LEN = buff_len;
		
		//SoftwareSerial::gpsSerial(GPS_TX, GPS_RX);         // defines serial(UART) communication with GPS

}

// GPS initiaization
void zGPS::init(){
	GPSBufferLength=0;
	NMEADivLen=0;
	
	GPSBuffer = malloc(GPS_BUFF_LEN*sizeof(byte));

  gpsSerial.setTimeout(1500);
  
	pinMode(GPS_ENABLE, OUTPUT);
	digitalWrite(GPS_ENABLE, HIGH);
	// TO-DO: protect init
	gpsSerial.begin(GPS_BAUD_RATE);
	#ifdef DEBUG_MODE
		Serial.println(F("GPS PINS:"));
		Serial.print(F("GPS TX:"));Serial.println(GPS_TX);
		Serial.print(F("GPS RX:"));Serial.println(GPS_RX);
		Serial.print(F("GPS EN:"));Serial.println(GPS_ENABLE);
		Serial.print(F("GPS BAUD:"));Serial.println(GPS_BAUD_RATE);
		Serial.print(F("GPS BUFF:"));Serial.println(GPS_BUFF_LEN);
		Serial.println();
	#endif
}
		
// PRIVATE FUNCTIONS
err_t zGPS::readStringBuffer(char* str, int32_t st_index, int32_t N){
  // this function stores values from the GPSBuffer in the "str" string
  // reads "N" ascii characters, starting from "st_index"
  if(N < 0)
    return INDEX_ERR;

  for(int i = 0; i < N; i++){
    str[i] = GPSBuffer[st_index+i];
  }

  str[N] = '\0';
  return NO_ERR;
}

int32_t zGPS::readParameter(uint8_t par_id){
	uint8_t i;
	int32_t val = 0;
	// Check validity of information
	if(par_id >= NMEADivLen)
		return 0;
	
	for(i = NMEADiv[par_id] + 1; i < NMEADiv[par_id+1]; i++){
		// Ignores '.' chars
		if(GPSBuffer[i] == '.')
			continue;
		
		if ((GPSBuffer[i] < '0') || (GPSBuffer[i] > '9'))
			return 0;
		
		// Increments val
		val = val*10;
		val += GPSBuffer[i] - '0';
	}
	
	return val;
}

void zGPS::setNMEADiv(char separator){
  NMEADivLen = 0;
  for(int32_t i = 0; i < GPSBufferLength; i++)
    if(GPSBuffer[i] == separator)
      NMEADiv[NMEADivLen++] = i;
}

void zGPS::ProcessGGA(){
	// Processes a NMEA message with "GGA" Mnemonic and attributes value for corresponding global variables
	// this function is called by "GPS_Process_NMEA_Line"
	// the message must be in the GPSBuffer before this functions calling
	int32_t updated_val;
	setNMEADiv(',');

	//printString(GPSBuffer, GPSBufferLength);
	//TO-DO: check an handle error in each read
	// time (NMEADiv[0]~NMEADiv[1])
	updated_val = readParameter(0);
	if(updated_val)
		Time = updated_val;

	// latitude module (NMEADiv[1]~NMEADiv[2])
	updated_val = readParameter(1);
	if(updated_val)
		lat = updated_val;
	
	// N/S (NMEADiv[2]~NMEADiv[3])
  if(GPSBuffer[NMEADiv[2]+1] == 'N' || GPSBuffer[NMEADiv[2]+1] == 'S')
    NS = GPSBuffer[NMEADiv[2]+1];
    
	switch(NS){
		case 'N':
			lat = absolute(lat);		// sets latitude to be positive
		break;
		case 'S':
			lat = -absolute(lat);	// sets latitude to be negative
		break;
	}
      
	// longitude (NMEADiv[3]~NMEADiv[4])
	updated_val = readParameter(3);
	if(updated_val)
		lon = updated_val;

	// E/W (NMEADiv[4]~NMEADiv[5])
  if(GPSBuffer[NMEADiv[4]+1] == 'E' || GPSBuffer[NMEADiv[4]+1] == 'W')
    EW = GPSBuffer[NMEADiv[4]+1];
    
	switch(EW){
		case 'E':
			lon = absolute(lon);		// sets longitude to be positive
		break;
		case 'W':
			lon = -absolute(lon);	// sets longitude to be negative
		break;
	}
	
	// numSat (NMEADiv[6]~NMEADiv[7])
	updated_val = readParameter(6);
	if(updated_val)
		numSat = updated_val;

	// horizontal precision(HDOP) (NMEADiv[7]~NMEADiv[8])
	updated_val = readParameter(7);
	if(updated_val)
		Hdop = updated_val;

	// altitude (NMEADiv[8]~NMEADiv[9])
	updated_val = readParameter(8);
	if(updated_val)
		alt = updated_val;

  // info age (NMEADiv[12]~NMEADiv[13])
  updated_val = readParameter(12);
  if(updated_val)
    age = updated_val;
    
	#ifdef DEBUG_MODE_HIGH
		Serial.println(F("GGA processing results:"));
		Serial.print(F("time: "));Serial.println(Time);
		Serial.print(F("latitude: "));Serial.println(lat);
		Serial.print(F("longitude: "));Serial.println(lon);
		Serial.print(F("number of satellites: "));Serial.println(numSat);
		Serial.print(F("horizontal precision: "));Serial.println(Hdop);
		Serial.print(F("altitude: "));Serial.println(alt);
		Serial.print(F("information age: "));Serial.println(age);
	#endif
}

void zGPS::ProcessRMC(){
  // Processes a NMEA message with "GGA" Mnemonic and attributes value for corresponding global variables
  // this function is called by "GPS_Process_NMEA_Line"
  // the message must be in the GPSBuffer before this functions calling
  int32_t updated_val;
  setNMEADiv(',');

  //printString(GPSBuffer, GPSBufferLength);
  //TO-DO: check an handle error in each read
  // time (NMEADiv[0]~NMEADiv[1])
  updated_val = readParameter(0);
  if(updated_val)
    Time = updated_val;

  // latitude module (NMEADiv[2]~NMEADiv[3])
  updated_val = readParameter(2);
  if(updated_val)
    lat = updated_val;
  
  // N/S (NMEADiv[3]~NMEADiv[4])
  if(GPSBuffer[NMEADiv[3]+1] == 'N' || GPSBuffer[NMEADiv[3]+1] == 'S')
    NS = GPSBuffer[NMEADiv[3]+1];
    
  switch(NS){
    case 'N':
      lat = absolute(lat);    // sets latitude to be positive
    break;
    case 'S':
      lat = -absolute(lat); // sets latitude to be negative
    break;
  }
      
  // longitude (NMEADiv[4]~NMEADiv[5])
  updated_val = readParameter(4);
  if(updated_val)
    lon = updated_val;

  // E/W (NMEADiv[5]~NMEADiv[6])
  if(GPSBuffer[NMEADiv[5]+1] == 'E' || GPSBuffer[NMEADiv[5]+1] == 'W')
    EW = GPSBuffer[NMEADiv[5]+1];
    
  switch(EW){
    case 'E':
      lon = absolute(lon);    // sets longitude to be positive
    break;
    case 'W':
      lon = -absolute(lon); // sets longitude to be negative
    break;
  }
  
  // speed (NMEADiv[6]~NMEADiv[7])
  updated_val = readParameter(6);
  if(updated_val)
    spd = (updated_val*1,852);    // Multiply by 1,852 to convert from knots to kph

  // date (NMEADiv[8]~NMEADiv[9])
  updated_val = readParameter(8);
  if(updated_val)
    date = updated_val;
  
  #ifdef DEBUG_MODE_HIGH
    Serial.println(F("RMC processing results:"));
    Serial.print(F("time: "));Serial.println(Time);
    Serial.print(F("latitude: "));Serial.println(lat);
    Serial.print(F("longitude: "));Serial.println(lon);
    Serial.print(F("speed: "));Serial.println(spd);
    Serial.print(F("date: "));Serial.println(date);
  #endif
}

void zGPS::ProcessGLL(){
  // Processes a NMEA message with "GLL" Mnemonic and attributes value for corresponding global variables
  // this function is called by "GPS_Process_NMEA_Line"
  // the message must be in the GPSBuffer before this functions calling
  int32_t updated_val;
  setNMEADiv(',');

  // latitude module (NMEADiv[0]~NMEADiv[1])
  updated_val = readParameter(0);
  if(updated_val)
    lat = updated_val;
  
  // N/S (NMEADiv[1]~NMEADiv[2])
  if(GPSBuffer[NMEADiv[1]+1] == 'N' || GPSBuffer[NMEADiv[1]+1] == 'S')
    NS = GPSBuffer[NMEADiv[1]+1];
    
  switch(NS){
    case 'N':
      lat = absolute(lat);    // sets latitude to be positive
    break;
    case 'S':
      lat = -absolute(lat); // sets latitude to be negative
    break;
  }
      
  // longitude (NMEADiv[2]~NMEADiv[3])
  updated_val = readParameter(2);
  if(updated_val)
    lon = updated_val;

  // E/W (NMEADiv[3]~NMEADiv[4])
  if(GPSBuffer[NMEADiv[3]+1] == 'E' || GPSBuffer[NMEADiv[3]+1] == 'W')
    EW = GPSBuffer[NMEADiv[3]+1];
    
  switch(EW){
    case 'E':
      lon = absolute(lon);    // sets longitude to be positive
    break;
    case 'W':
      lon = -absolute(lon); // sets longitude to be negative
    break;
  }
  
  // time (NMEADiv[4]~NMEADiv[5])
  updated_val = readParameter(4);
  if(updated_val)
    Time = updated_val;
  
  #ifdef DEBUG_MODE_HIGH
    Serial.println(F("GLL processing results:"));
    Serial.print(F("time: "));Serial.println(Time);
    Serial.print(F("latitude: "));Serial.println(lat);
    Serial.print(F("longitude: "));Serial.println(lon);
  #endif
}

void zGPS::ProcessGNS(){
  // Processes a NMEA message with "GNS" Mnemonic and attributes value for corresponding global variables
  // this function is called by "GPS_Process_NMEA_Line"
  // the message must be in the GPSBuffer before this functions calling
  int32_t updated_val;
  setNMEADiv(',');
  
  // time (NMEADiv[0]~NMEADiv[1])
  updated_val = readParameter(0);
  if(updated_val)
    Time = updated_val;

  // latitude module (NMEADiv[1]~NMEADiv[2])
  updated_val = readParameter(1);
  if(updated_val)
    lat = updated_val;
  
  // N/S (NMEADiv[2]~NMEADiv[3])
  if(GPSBuffer[NMEADiv[2]+1] == 'N' || GPSBuffer[NMEADiv[2]+1] == 'S')
    NS = GPSBuffer[NMEADiv[2]+1];
    
  switch(NS){
    case 'N':
      lat = absolute(lat);    // sets latitude to be positive
    break;
    case 'S':
      lat = -absolute(lat); // sets latitude to be negative
    break;
  }
      
  // longitude (NMEADiv[3]~NMEADiv[4])
  updated_val = readParameter(3);
  if(updated_val)
    lon = updated_val;

  // E/W (NMEADiv[4]~NMEADiv[5])
  if(GPSBuffer[NMEADiv[4]+1] == 'E' || GPSBuffer[NMEADiv[4]+1] == 'W')
    EW = GPSBuffer[NMEADiv[4]+1];
    
  switch(EW){
    case 'E':
      lon = absolute(lon);    // sets longitude to be positive
    break;
    case 'W':
      lon = -absolute(lon); // sets longitude to be negative
    break;
  }
  
  // numSat (NMEADiv[6]~NMEADiv[7])
  updated_val = readParameter(6);
  if(updated_val)
    numSat = updated_val;

  // horizontal precision(HDOP) (NMEADiv[7]~NMEADiv[8])
  updated_val = readParameter(7);
  if(updated_val)
    Hdop = updated_val;

  // altitude (NMEADiv[8]~NMEADiv[9])
  updated_val = readParameter(8);
  if(updated_val)
    alt = updated_val;

  // info age (NMEADiv[10]~NMEADiv[11])
  updated_val = readParameter(10);
  if(updated_val)
    age = updated_val;
    
  #ifdef DEBUG_MODE_HIGH
    Serial.println(F("GNS processing results:"));
    Serial.print(F("time: "));Serial.println(Time);
    Serial.print(F("latitude: "));Serial.println(lat);
    Serial.print(F("longitude: "));Serial.println(lon);
    Serial.print(F("number of satellites: "));Serial.println(numSat);
    Serial.print(F("horizontal precision: "));Serial.println(Hdop);
    Serial.print(F("altitude: "));Serial.println(alt);
  #endif
}
// USER FUNCTIONS

err_t zGPS::GPS_update(){
  // Reads a NMEA message from GPS and stores it in GPSBuffer
  int inByte;                    // Byte recieved from GPS
  int32_t aux = 0;
  unsigned long t;               // Variable for controling time spent in this function
  
  t = millis();
  GPSBufferLength = 0;
  gpsSerial.flush();
  #ifdef DEBUG_MODE
    Serial.println(F("Raw NMEA data:"));
  #endif
  
  while(millis() < t + GPS_UPDATE_TIME){
    while (gpsSerial.available() > 0){
    if(aux >= GPS_BUFF_LEN)
      return READ_ERR;

    inByte = gpsSerial.read();
    
    #ifdef DEBUG_MODE
      Serial.write(inByte);                         // Output raw NMEA string from GPS.
    #endif
    #ifdef DEBUG_GPS
      Serial.write(F("  "));
      Serial.print(aux, DEC);
      Serial.write(F("  "));
      Serial.println((inByte != '$' && !aux));
    #endif
    
    if (inByte != '$' && !GPSBufferLength)
      continue;
    
    if (inByte == '$')
      GPSBufferLength = 0;
    if (inByte != '\r')
      GPSBuffer[GPSBufferLength++] = inByte;
    if (inByte == '\n'){
      //GPSBufferLength = aux;
      //aux = 0;
      return NO_ERR;
    }
    }
  #ifdef DEBUG_GPS
    Serial.println(F("GPS unnavaliable"));
  #endif
  }
  return INV_DATA_ERR;
}

/*
err_t zGPS::GPS_update()
{
  int inByte;
  String b;
  GPSBufferLength = 0;
  Serial.println(F("Flushing buffer"));
  gpsSerial.flush();
  Serial.println(F("Searching for '$'"));
  while ((inByte = gpsSerial.read()) != '$')
    if (inByte == -1)
      return INV_DATA_ERR;
  Serial.println(F("'$' found"));
  b = gpsSerial.readStringUntil('\n');
  //GPSBufferLength = gpsSerial.readBytesUntil('\n', GPSBuffer, 82);
  Serial.print(F("BUFFER:"));
  Serial.println(b);
  /*
  while (gpsSerial.available() > 0){
    inByte = gpsSerial.read();
    //Serial.write(inByte);                         // Output raw NMEA string from GPS. Use for debugging
    if ((inByte =='$') || (GPSBufferLength >= 80))
      GPSBufferLength = 0;
    if (inByte != '\r')
      GPSBuffer[GPSBufferLength++] = inByte;
    if (inByte == '\n'){
      printString(GPSBuffer, GPSBufferLength);
      return NO_ERR;
    }
  }
  
  return INV_DATA_ERR;
  //Serial.println(F("GPS unnavaliable"));
}
*/
err_t zGPS::GPS_Process_NMEA_Line(){
  // Read NMEA message TalkerId and Mnemonic and process it accordingly
  // the message must be in the GPSBuffer before this functions calling
  //
  // NMEA talker ID reference table:
  //  ______________________
  // |    GNSS     |TalkerID|
  // |-------------|--------|
  // |GPS,SBAS,QZSS|   GP   |
  // |   GLONASS   |   GL   |
  // |   Galileo   |   GA   |
  // |   BeiDou    |   GB   |
  // |    any      |   GN   |
  //  ----------------------
  readStringBuffer(NMEATalkerID, 1, 2);           // get NMEA talker ID
  readStringBuffer(NMEAMnemonic, 3, 3);           // get NMEA mnemonic that defines message parameters
  //Serial.println(NMEAMnemonic);
  if(!strcmp(NMEAMnemonic,"GGA"))                 // process NMEA message with "GGA" Mnemonic (implementar outras mensagens)
    ProcessGGA();
  else if(!strcmp(NMEAMnemonic,"RMC"))                 // process NMEA message with "RMC" Mnemonic (implementar outras mensagens)
    ProcessRMC();
  else if(!strcmp(NMEAMnemonic,"GLL"))                 // process NMEA message with "RMC" Mnemonic (implementar outras mensagens)
    ProcessGLL();
  else if(!strcmp(NMEAMnemonic,"GNS"))                 // process NMEA message with "RMC" Mnemonic (implementar outras mensagens)
    ProcessGNS();
  else
    return INV_DATA_ERR;
  
  return NO_ERR;
}

err_t zGPS::read(){
  err_t error_value;

  error_value = GPS_update();
  
  if(error_value != NO_ERR){
		#ifdef DEBUG_MODE_HIGH
			Serial.print(F("Error while updating. Error type: ")); Serial.println(error_value);
		#endif
    return error_value;
  }
	#ifdef DEBUG_MODE_HIGH
		else
			Serial.println(F("GPS Updated sucessfully with buffer:"));
			printString(GPSBuffer, GPSBufferLength);
	#endif
  error_value = GPS_Process_NMEA_Line();
	#ifdef DEBUG_MODE
		if(error_value != NO_ERR)
			Serial.print(F("Error while processing. Error type: ")); Serial.println(error_value);
	#endif
  return error_value;
}




// Airborne mode methods

void zGPS::setGPS_DynamicMode6(){
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };

  #ifdef DEBUG_MODE_HIGH
    Serial.println();Serial.println(F("Routine to set GPS to airborne mode started."));Serial.println();
  #endif
  
  while(!gps_set_sucess){
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
  
  #ifdef DEBUG_MODE_HIGH
    Serial.println();Serial.println(F("GPS set to airborne mode sucesfully"));Serial.println();
  #endif
}

void zGPS::sendUBX(uint8_t *MSG, uint8_t len) {
  gpsSerial.flush();
  gpsSerial.write(0xFF);
  delay(500);
  
  #ifdef DEBUG_MODE
    Serial.println(F("Message sent:"));
  #endif
  
  for(int i=0; i<len; i++){
    gpsSerial.write(MSG[i]);
    #ifdef DEBUG_MODE
      Serial.print(MSG[i],HEX);Serial.print(F(" "));
    #endif
  }
  #ifdef DEBUG_MODE
    Serial.println();
  #endif
}

boolean zGPS::getUBX_ACK(uint8_t *MSG) {
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

  #ifdef DEBUG_MODE
    Serial.println(F("Message recieved:"));
  #endif
  while (1) {
    // Test for success
    if (ackByteID > 9){
      #ifdef DEBUG_MODE
        Serial.println();
      #endif
      return true;      // All packets in order!
    }
    
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 1500){
      #ifdef DEBUG_MODE
        Serial.println();Serial.println(F("Acknowledgement timed out"));
      #endif
      return false;
    }
 
    // Make sure data is available to read
    if (gpsSerial.available()) {
      b = gpsSerial.read();
      #ifdef DEBUG_MODE
        Serial.print(b, HEX);Serial.print(F(" "));
      #endif
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID])
        ackByteID++;
      else
        ackByteID = 0; // Reset and look again, invalid order
    }
  }
  #ifdef DEBUG_MODE
    Serial.println();
  #endif
}