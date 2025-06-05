#include "motor.h"

Motor::Motor(uint8_t motorPin) : pin(motorPin) {}

void Motor::init(){
  // configure pins as output
  pinMode(this->pin, OUTPUT);
}

void Motor::write(int pulse){
  /* create valid duty cycle value (% time on vs total period) as a fraction of 255 (8 bit max)
   - servos and esc input utilizes 1ms - 2ms pulse width duration
   - for ~ 490 Hz (488) the period is ~2.04 ms
   - duty cycle for 1ms (0% throttle) is 1/2.04 which is 49%, and for 2ms duty cycle is 98%
   - analogWrite requires integer from 0 to 255 (8 bit) which will represent duty cycle of pwm output
   - so 1-2ms pulse width durations correspond to an integer value between 125 (49% of 255) and 250 (98% of 255)*/
  int constrainedPulse = constrain(pulse,1000,2000);
  int val = map(constrainedPulse,1000,2000,125,250); // map throttle to 8 bit value such that analogwrite can send a valid signal
  analogWrite(this->pin,val); // send pulse
}