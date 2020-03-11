#include <Servo.h>

#define SERVO_COUNT 54
Servo servoArray[SERVO_COUNT];

void setup() {
  Serial.begin(115200);
  for(int i=0;i<=53;++i){
    servoArray[i].attach(i);
  }
}

unsigned char readByte(){
  char buffer;
  while(Serial.readBytes(&buffer, 1) == 0){} //TODO: add safety shutoff
  return static_cast<unsigned char>(buffer);
}

void loop() {
  int isOn = 0;
  
  while(true){
    unsigned char opCode = readByte();
    switch(opCode){
      case 0x1: //motor change
        byte pinId = readByte();
        byte intensity = readByte();
        //analogWrite(pinId, intensity);
        //servoArray[pinId].writeMicroseconds((((int)intensity)-180)*7.14+1500);
        //servoArray[pinId].writeMicroseconds((intensity/(250.0-110.0))*(1900-1100)+1100);
        servoArray[pinId].writeMicroseconds((intensity/255.0)*1000+1000);
        Serial.print("Motor Message (pin: ");
        Serial.print(pinId);
        Serial.print(", intensity: ");
        Serial.print(intensity);
        Serial.println(") - arduino"); 
        break;
    }
  }
}

