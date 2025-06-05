#include "receiver.h"
#include "motor.h"

Receiver rx;

Motor FL(11);
Motor FR(3);
Motor BL(10);
Motor BR(9);

ISR(PCINT0_vect){
  rx.handleInterrupt();
}

void setup() {
  // put your setup code here, to run once:
  FL.init();
  FR.init();
  BL.init();
  BR.init();

  rx.init();

  FL.write(1000);
  FR.write(1000);
  BL.write(1000);
  BR.write(1000);

  delay(4000);

  FL.write(1000);
  FR.write(1000);
  BL.write(1000);
  BR.write(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  int throttle = rx.getThrottle();

  FL.write(throttle);
  FR.write(throttle);
  BL.write(throttle);
  BR.write(throttle);
}
