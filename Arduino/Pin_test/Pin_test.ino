int incomingByte = 0;
int port, i, pc;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Velocidade do serial
}

void loop() {
  i = 0;
  pc = port;
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) {
    if(!i)
      port = 0;
    // lê do buffer o dado recebido:
    incomingByte = Serial.read();

    if(incomingByte == 10)
      break;

    port *= 10;
    incomingByte -= 48;
    port += incomingByte;
    i++;
    delay(200);
  }
  if(pc != port){
    Serial.print("Pino acionado: ");
    Serial.println(port, DEC);
    
    pinMode(port, OUTPUT);
    digitalWrite(port, HIGH);
    digitalWrite(pc, LOW);
  }
}
