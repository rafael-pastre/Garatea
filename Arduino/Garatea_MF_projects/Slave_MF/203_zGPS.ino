// Verificar tamanho da parametros da mensagem NMEA
// Implementar checksum
// Implementar outros tipos de mensagem


zGPS::zGPS(uint8_t tx, uint8_t rx, uint32_t baud, uint16_t buff_len):gpsSerial(tx, rx){
		// pin atribution
		GPS_TX = tx;
		GPS_RX = rx;
		GPS_ENABLE = ALWAYS_ENABLED;
		GPS_BAUD_RATE = baud;
		GPS_BUFF_LEN = buff_len;
		
		//SoftwareSerial::gpsSerial(GPS_TX, GPS_RX);         // defines serial(UART) communication with GPS

}

zGPS::zGPS(uint8_t tx, uint8_t rx, uint8_t en, uint32_t baud, uint16_t buff_len):gpsSerial(tx, rx){
		// pin atribution
		GPS_TX = tx;
		GPS_RX = rx;
		GPS_ENABLE = en;
		GPS_BAUD_RATE = baud;
		GPS_BUFF_LEN = buff_len;
		
		//SoftwareSerial::gpsSerial(GPS_TX, GPS_RX);         // defines serial(UART) communication with GPS

}

void zGPS::init(){
	GPS_hour=0, GPS_min=0, GPS_sec=0;         // time
	GPS_lat_int=0, GPS_lat_frac=0;            // latitude
	GPS_NS = 'X';                             // north/south
	GPS_long_int=0, GPS_long_frac=0;          // longitude
	GPS_EW = 'X';                             // east/west
	GPS_quality='0';                          // quality
	GPS_Satellites=0;                         // number of satelites
	GPS_HDOP_int=0, GPS_HDOP_frac=0;          // horizontal precision
	GPS_Altitude=0;                           // altitude
	GPS_age_int=0, GPS_age_frac=0;            // information age
	
	GPSBufferLength=0;
	NMEADivLen=0;
	
	GPSBuffer = malloc(GPS_BUFF_LEN*sizeof(byte));
	
	pinMode(GPS_ENABLE, OUTPUT);
	digitalWrite(GPS_ENABLE, HIGH);
	// TO-DO: protect init
	gpsSerial.begin(GPS_BAUD_RATE);
	#ifdef DEBUG_MODE
		Serial.println("GPS PINS:");
		Serial.print("GPS TX:");Serial.println(GPS_TX);
		Serial.print("GPS RX:");Serial.println(GPS_RX);
		Serial.print("GPS EN:");Serial.println(GPS_ENABLE);
		Serial.print("GPS BAUD:");Serial.println(GPS_BAUD_RATE);
		Serial.print("GPS BUFF:");Serial.println(GPS_BUFF_LEN);
		Serial.println("GPS PINS:");
	#endif
}

uint32_t zGPS::time(){
	return GPS_hour;
}
int32_t zGPS::lat(){
	return GPS_lat_int;
}
int32_t zGPS::lon(){
	return GPS_long_int;
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

err_t zGPS::readBufferIndex(int32_t st_index, int32_t end_index, char ignore, int32_t* val_addr){
  // this function reads a integer value from the buffer and stores it in val_addr
  // each digit of the value read must be coded in ascii format
  // "st_index" is the starting index and "end_index" is the ending index for reading
  // "ignore" characters will be ignored in reading
  // characters in "st_index" and "end_index" will be read
  int32_t aux;
  if(st_index > end_index)
    return INDEX_ERR;
    
  aux = 0;
  
  while(st_index <= end_index){
    if(GPSBuffer[st_index] == ignore){
      st_index++;
      continue;
    }
    if ((GPSBuffer[st_index] < '0') || (GPSBuffer[st_index] > '9'))
      return READ_ERR;

    aux = aux*10;
    aux += GPSBuffer[st_index] - '0';
    st_index++;
  }

  if(aux <= 0)
    return INV_DATA_ERR;

  *val_addr = aux;
  return NO_ERR;
}

void zGPS::setNMEADiv(char separator){
  NMEADivLen = 0;
  for(int32_t i = 0; i < GPSBufferLength; i++)
    if(GPSBuffer[i] == separator)
      NMEADiv[NMEADivLen++] = i;
}

err_t zGPS::ProcessGGA(){
  // Processes a NMEA message with "GGA" Mnemonic and attributes value for corresponding global variables
  // this function is called by "GPS_Process_NMEA_Line"
  // the message must be in the GPSBuffer before this functions calling
  int32_t aux;
  err_t err_return = NO_ERR;
  setNMEADiv(',');

  //printString(GPSBuffer, GPSBufferLength);
  //TO-DO: check an handle error in each read
  // time (NMEADiv[0]~NMEADiv[1])
  readBufferIndex(NMEADiv[0]+1, NMEADiv[0]+2, NMEA_IGNR_NULL, &GPS_hour);         // hour (2 chars)
  readBufferIndex(NMEADiv[0]+3, NMEADiv[0]+4, NMEA_IGNR_NULL, &GPS_min);          // minutes (2 chars)
  readBufferIndex(NMEADiv[0]+5, NMEADiv[1]-1, '.', &GPS_sec);                     // seconds (ignore '.')
  
  // latitude (NMEADiv[1]~NMEADiv[2])
  readBufferIndex(NMEADiv[1]+1, NMEADiv[1]+2, NMEA_IGNR_NULL, &GPS_lat_int);      // integer part (2 chars)
  readBufferIndex(NMEADiv[1]+3, NMEADiv[2]-1, '.', &GPS_lat_frac);                // fractional part (ignore '.')

  // N/S (NMEADiv[2]~NMEADiv[3])
  if ((GPSBuffer[NMEADiv[2]+1] == 'N') || (GPSBuffer[NMEADiv[2]+1] == 'S'))
    GPS_NS = GPSBuffer[NMEADiv[2]+1];
  else
    err_return = INV_DATA_ERR;
      
  // longitude (NMEADiv[3]~NMEADiv[4])
  readBufferIndex(NMEADiv[3]+1, NMEADiv[3]+3, NMEA_IGNR_NULL, &GPS_long_int);     // integer part (3 chars)
  readBufferIndex(NMEADiv[3]+4, NMEADiv[4]-1, '.', &GPS_long_frac);               // fractional part (ignore '.')

  // E/W (NMEADiv[4]~NMEADiv[5])
  if ((GPSBuffer[NMEADiv[4]+1] == 'E') || (GPSBuffer[NMEADiv[4]+1] == 'W'))
    GPS_EW = GPSBuffer[NMEADiv[4]+1];
  else
    err_return = INV_DATA_ERR;

  // quality (NMEADiv[5]~NMEADiv[6])
  if ((GPSBuffer[NMEADiv[5]+1] >= '0') && (GPSBuffer[NMEADiv[5]+1] <= '9'))
    GPS_quality = GPSBuffer[NMEADiv[5]+1];
  else
    err_return = INV_DATA_ERR;
    
  // numSat (NMEADiv[6]~NMEADiv[7])
  readBufferIndex(NMEADiv[6]+1, NMEADiv[7]-1, NMEA_IGNR_NULL, &GPS_Satellites);

  // horizontal precision(HDOP) (NMEADiv[7]~NMEADiv[8])
  aux = findStrIndex(GPSBuffer, NMEADiv[7]+1, NMEADiv[8]-1, '.');
  if(aux > 0){
    readBufferIndex(NMEADiv[7]+1, aux-1, NMEA_IGNR_NULL, &GPS_HDOP_int);
    readBufferIndex(aux+1, NMEADiv[8]-1, NMEA_IGNR_NULL, &GPS_HDOP_frac);
  }  
  else
    err_return = INV_DATA_ERR;

  // altitude (NMEADiv[8]~NMEADiv[9])
  aux = findStrIndex(GPSBuffer, NMEADiv[8]+1, NMEADiv[9]-1, '.');
  if(aux > 0){
    readBufferIndex(NMEADiv[8]+1, aux-1, NMEA_IGNR_NULL, &GPS_Altitude);
    //readBufferIndex(aux+1, NMEADiv[9]-1, NMEA_IGNR_NULL, &GPS_HDOP_frac);
  }  
  else
    err_return = INV_DATA_ERR;

  //GPS_print_Info();
  return err_return;
}

// USER FUNCTIONS
err_t zGPS::GPS_update(){
  // Reads a NMEA message from GPS and stores it in GPSBuffer
  int inByte;										// Byte recieved from GPS
  int32_t aux = 0;
  unsigned long time;								// Variable for controling time spent in this function
  
  time = millis();
  
	#ifdef DEBUG_MODE
		Serial.println("Raw NMEA data:");
	#endif
	
  while(millis() < time + GPS_UPDATE_TIME){
	  while (gpsSerial.available() > 0){
		if(aux >= GPS_BUFF_LEN)
		  return READ_ERR;

		inByte = gpsSerial.read();
		
		#ifdef DEBUG_MODE
			Serial.write(inByte);                         // Output raw NMEA string from GPS.
		#endif
		#ifdef DEBUG_GPS
			Serial.write("  ");
			Serial.print(aux, DEC);
			Serial.write("  ");
			Serial.println((inByte != '$' && !aux));
		#endif
		
		if (inByte != '$' && !aux)
		  continue;
		
		if (inByte == '$')
		  aux = 0;
		if (inByte != '\r')
		  GPSBuffer[aux++] = inByte;
		if (inByte == '\n'){
		  GPSBufferLength = aux;
		  aux = 0;
		  return NO_ERR;
		}
	  }
	#ifdef DEBUG_GPS
		Serial.println("GPS unnavaliable");
	#endif
  }
  return INV_DATA_ERR;
}

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
  else
    return INV_DATA_ERR;
  
  return NO_ERR;
}

/**
err_t zGPS::GPS_init(){
  
  return NO_ERR;
}
*/

err_t zGPS::read(){
  err_t error_value;
  error_value = GPS_update();
  if(error_value != NO_ERR){
		#ifdef DEBUG_MODE
			Serial.print("Error while updating. Error type: "); Serial.println(error_value);
		#endif
    return error_value;
  }
	#ifdef DEBUG_MODE
		else
			Serial.println("GPS Updated sucessfully with buffer:");
			printString(GPSBuffer, GPSBufferLength);
	#endif
  error_value = GPS_Process_NMEA_Line();
	#ifdef DEBUG_MODE
		if(error_value != NO_ERR)
			Serial.print("Error while processing. Error type: "); Serial.println(error_value);
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

  #ifdef DEBUG_MODE
    Serial.println();Serial.println("Routine to set GPS to airborne mode started.");Serial.println();
  #endif
  
  while(!gps_set_sucess){
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
  
  #ifdef DEBUG_MODE
    Serial.println();Serial.println("GPS set to airborne mode sucesfully");Serial.println();
  #endif
}

void zGPS::sendUBX(uint8_t *MSG, uint8_t len) {
  gpsSerial.flush();
  gpsSerial.write(0xFF);
  delay(500);
  
  #ifdef DEBUG_MODE
    Serial.println("Message sent:");
  #endif
  
  for(int i=0; i<len; i++){
    gpsSerial.write(MSG[i]);
    #ifdef DEBUG_MODE
      Serial.print(MSG[i],HEX);Serial.print(" ");
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
    Serial.println("Message recieved:");
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
    if (millis() - startTime > 3000){
      #ifdef DEBUG_MODE
        Serial.println();Serial.println("Acknowledgement timed out");
      #endif
      return false;
    }
 
    // Make sure data is available to read
    if (gpsSerial.available()) {
      b = gpsSerial.read();
      #ifdef DEBUG_MODE
        Serial.print(b, HEX);Serial.print(" ");
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
// DEBUG
