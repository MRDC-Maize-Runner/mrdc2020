void setup() {
  Serial.begin(9600);
  for(int i=0;i<=53;++i){
    pinMode(i, OUTPUT);
  }
}

byte readByte(){
  byte buffer;
  while(Serial.readBytes(&buffer, 1) == 0){} //TODO: add safety shutoff
  Serial.write(buffer);
  return buffer;
}

void loop() {
  int isOn = 0;
  
  while(true){
    byte opCode = readByte();
    switch(opCode){
      case 0x1: //motor change
        byte pinId = readByte();
        byte intensity = readByte();
        analogWrite(pinId, intensity);
        break;
    }
  }
}
