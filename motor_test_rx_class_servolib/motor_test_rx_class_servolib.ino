#include "receiver.h"
#include <Servo.h>

Receiver rx;

Servo FL;
Servo FR;
Servo BL;
Servo BR;


ISR(PCINT0_vect){
  rx.handleInterrupt();
}
void setup() {
  // put your setup code here, to run once:
  rx.init();
  
  FL.attach(11);
  FR.attach(3);
  BL.attach(10);
  BR.attach(9);

  FR.writeMicroseconds(1000);
  delay(4000);
  FR.writeMicroseconds(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  int throttle = rx.getThrottle();

  FR.writeMicroseconds(throttle);
}
