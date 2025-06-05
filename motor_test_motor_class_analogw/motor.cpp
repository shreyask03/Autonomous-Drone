#include "motor.h"

Motor::Motor(uint8_t motorPin) : pin(motorPin) {}

void Motor::init(){
  pinMode(this->pin, OUTPUT);
}

void Motor::write(int pulse){
  int val = map(pulse,1000,2000,125,255); // map throttle to 8 bit value such that analogwrite can send a valid signal
  analogWrite(this->pin,val); // send pulse
}