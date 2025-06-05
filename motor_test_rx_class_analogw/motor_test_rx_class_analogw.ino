#include "receiver.h"

Receiver rx;

ISR(PCINT0_vect){
  rx.handleInterrupt();
}

void setup() {
  // put your setup code here, to run once:

  rx.init();


  pinMode(11,OUTPUT); // front left
  pinMode(3,OUTPUT); // front right
  pinMode(10,OUTPUT); // back left
  pinMode(9,OUTPUT); // back right

  /* arduino writes hardware pwm on pins 3,9,10,11 at 490 Hz, pins 5,6 at 980 Hz */
  // 490hz pwm period is ~2.04 ms
  // 1ms pulse width then is ~49% duty cycle pwm
  // 2ms pulse width then is ~98% duty cycle pwm
  // analogWrite() sends 8 bit value (0-255) to encode duty cycle of pwm
  // for 1ms (off) write 0.49 * 255 ~= 125
  // for 2ms (on) write 0.98 * 255 ~= 250

  analogWrite(3,125);
  delay(4000);
  analogWrite(3,125);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  int throttle = rx.getThrottle();

  int val = map(throttle,1000,2000,125,250); // map throttle to 8 bit value such that analogwrite can send a valid signal

  analogWrite(3,val);

}
