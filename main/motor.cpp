#include "motor.h"


Motor::Motor(uint8_t motorPin, uint16_t minPulseUs = 1000, uint16_t maxPulseUs = 2000) : pin(motorPin), minPulse(minPulseUs), maxPulse(maxPulseUs),throttleUs(minPulseUs){
    pinMode(pin,OUTPUT);
}

void Motor::init(){
  // set up timers to generate 488 Hz PWM 
  if(this->pin == 3 || this->pin == 11){
    // set up 488 Hz PWM generation on timer 2
    // timer 2 outputs PWM signal on OC2A and OC2B, or pins D11 and D3 respectively
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2A |= (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // set non-inverting, fast pwm mode for both A and B with TOP = 255
    TCCR2B |= (1 << CS22) | (1 << CS20); // set prescalar to 128,  do this last, as it starts clock

  }
  else if(this->pin == 9 || this->pin == 10){
    // set up 488 Hz PWM generation on timer 1
    // timer 1 outputs PWM signal on OC1A and OC1B, or pins D9 and D10 respectively
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // set non-inverting, fast pwm mode for both A and B 
    TCCR1B |= (1 << WGM13) | (1 << WGM12); // set ICR1 register as TOP so it can be customized without removing a PWM pin from use

    // set ICR1 to 128 to get 488 Hz pwm with 256 prescaler
    ICR1 = 128; // 0x0080 in hexadecimal base 16

    TCCR1B |= (1 << CS12); // set prescalar to 256, do this last, as it starts clock
  }
  // note *** 1ms - 2ms RC pulse at 500hz requires 50% duty cycle signal for 1ms, 100% duty cycle for 2ms
}
void Motor::setPulse(uint16_t us){
  this->throttleUs = constrain(us, this->minPulse, this->maxPulse);
}

void Motor::writePulse(){

  // start clock here to avoid premature motor arming and weird glitches when calling init()
  // TCCR2B |= (1 << CS22) | (1 << CS20); // set prescalar to 128,  do this last, as it starts clock
  // TCCR1B |= (1 << CS12); // set prescalar to 256, do this last, as it starts clock


  // send a PWM pulse to the corresponding pin
  if(this->pin == 9){
    // OCR1A = map(this->throttleUs, this->minPulse, this->maxPulse, 64, 128 );
    OCR1A = map(this->throttleUs, this->minPulse, this->maxPulse, 62, 125 ); // true 1ms - 2ms pulse widths in a 2.048 ms pwm period in case exactness needed
  }
  else if(this->pin == 10){
    // OCR1B = map(this->throttleUs, this->minPulse, this->maxPulse, 64, 128 );
    OCR1B = map(this->throttleUs, this->minPulse, this->maxPulse, 62, 125 ); // true 1ms - 2ms pulse widths in a 2.048 ms pwm period in case exactness needed
  }
  else if(this->pin == 3){
    // OCR2B = map(this->throttleUs, this->minPulse, this->maxPulse, 127, 255 );
    OCR2B = map(this->throttleUs, this->minPulse, this->maxPulse, 124, 249 ); // true 1ms - 2ms pulse widths in a 2.048 ms pwm period in case exactness needed
  }
  else if(this->pin == 11){
    // OCR2A = map(this->throttleUs, this->minPulse, this->maxPulse, 127, 255 );
    OCR2A = map(this->throttleUs, this->minPulse, this->maxPulse, 124, 249 ); // true 1ms - 2ms pulse widths in a 2.048 ms pwm period in case exactness needed
  }
}