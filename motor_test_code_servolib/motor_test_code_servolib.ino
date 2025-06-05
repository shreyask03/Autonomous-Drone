#include <Servo.h>

Servo FL;
Servo FR;
Servo BL;
Servo BR;

unsigned long now = 0;
unsigned long prevTime = 0;

void setup() {
  // put your setup code here, to run once:
  
  FL.attach(11);
  FR.attach(3);
  BL.attach(10);
  BR.attach(9);

  FR.writeMicroseconds(1000);
  delay(4000);
  FR.writeMicroseconds(1000);
  // unsigned long now = millis();
  // if(now >= 20000UL){
  //   FR.writeMicroseconds(1000);
  // }

  // now = 0;
  // now = millis();
  // if(now >= 10000){
  //   // FL.writeMicroseconds(1000);
  //   FR.writeMicroseconds(1000);
  //   // BL.writeMicroseconds(1000);
  //   // BR.writeMicroseconds(1000);
  // }

}

void loop() {
  // put your main code here, to run repeatedly:
  now = millis();

  if(now - prevTime >= 10000UL){
    prevTime = now;

    FR.writeMicroseconds(1500);
  }



  // for(int i = 1; i <= 10; i++){
  //   FR.writeMicroseconds(100*i + 1000);
  // }

  // for(int i = 10; i >= 1; i--){
  //   FR.writeMicroseconds(2000 - (100*i));
  // }
  // now = 0;
  // now = millis();
  // if(now >= 10000UL){
  //   // FL.writeMicroseconds(1500);
  //   FR.writeMicroseconds(1500);
  //   // BL.writeMicroseconds(1500);
  //   // BR.writeMicroseconds(1500);
  // }

  // FL.writeMicroseconds(1000);
  // FR.writeMicroseconds(1000);
  // BL.writeMicroseconds(1000);
  // BR.writeMicroseconds(1000);

  // FL.writeMicroseconds(1250);
  // FR.writeMicroseconds(1250);
  // BL.writeMicroseconds(1250);
  // BR.writeMicroseconds(1250);

}
