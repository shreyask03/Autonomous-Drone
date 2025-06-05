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
  rx.init();

  rx.waitForConnect(); // wait for valid throttle signal
  // now safe to initialize motors
  FL.init();
  FR.init();
  BL.init();
  BR.init();
}

void loop() {
  // put your main code here, to run repeatedly:

  if(!rx.isArmed()){
    rx.waitForConnect();
  }
  else{
    int throttle = rx.getThrottle();
    FL.write(throttle);
    FR.write(throttle);
    BL.write(throttle);
    BR.write(throttle);
  }
}
