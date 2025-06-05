#include "receiver.h"

Receiver rx;

ISR(PCINT0_vect){
  rx.handleInterrupt();
}


void setup() {
  // put your setup code here, to run once:
  rx.init();

  /* register based esc understandable waveform generation */
  /* want to keep 490 Hz pwm (~ 488) on pins 3,9,10,11 */
  /* this corresponds to timer1 and timer2 output compare (OC) pins */ 

  // timer 1 is a 16 bit timer, with prescalars 1,8,64,256,1024
  // for both OC pins, set waveform generation on mode 14, fast pwm with ICR1 register as TOP value, with non-inverting pwm
  TCCR1A |= (1 << COM1A1) | (1 >> COM1B1) | (1 << WGM11);

  // finish setting OC pins to mode 14
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  ICR1 = 512; // set ICR1 as TOP, i.e. timer1 counts 513 ticks each cycle (zero-indexed); 
  

  // timer 2 is an 8 bit timer, with prescalars 1,8,32,64,128,256,1024
  // for both OC pins, set waveform generation to non-inverting fast pwm, mode 3 where TOP is MAX value (255 for 8 bit timer)
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);

  

  // finish timer configuration by setting prescalar, this is done last as it will start the timer
  // set timer 1 prescalar = 64, it will produce a ~ 488 Hz signal
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // set timer 2 prescalar = 128, it will produce a ~ 488 Hz signal
  TCCR2B |= (1 << CS22) | (1 << CS20);

  // send low signal to OC pins for both timers for arming
  OCR1A = 250; // ~ 1ms pulse width for a 488 Hz pwm signal where TOP is 512, i.e. 49% of 512
  OCR1B = 250;
  OCR2A = 125; // ~ 1ms pulse width for a 488 Hz pwm signal where TOP is 255, i.e. 49% of 255
  OCR2B = 125;

  // delay(4000);

  // TCCR1B |= (1 << CS11) | (1 << CS10);

  // // set timer 2 prescalar = 128, it will produce a ~ 488 Hz signal
  // TCCR2B |= (1 << CS22) | (1 << CS20);

  // // send low signal to OC pins for both timers for arming
  // OCR1A = 250; // ~ 1ms pulse width for a 488 Hz pwm signal where TOP is 512, i.e. 49% of 512
  // OCR1B = 250;
  // OCR2A = 125; // ~ 1ms pulse width for a 488 Hz pwm signal where TOP is 255, i.e. 49% of 255
  // OCR2B = 125;


}

void loop() {
  // put your main code here, to run repeatedly:
  int throttle = rx.getThrottle();

  // pulse width needs to remain at 1-2ms, which is ~ 49% to ~ 98% duty cycle of a ~ 490 hz signal
  // OCR1A and OCR1B for timer 1 need to be updated with throttle values as a fraction of the TOP which is set to 512
  int val1 = map(throttle,1000,2000,250,502);
  // OCR2A and OCR2B for timer 2 need to be updated with throttle values as a fraction of the TOP which is MAX, i.e. 255 for 8 bit timer
  int val2 = map(throttle,1000,2000,125,250); // same as previous analogWrite implementation

  // pulse goes high as timer starts counting from 0, once timer "x" hits val "x", the pulse goes low till period completes then repeats
  OCR1A = val1;
  OCR1B = val1;
  OCR2A = val2;
  OCR2B = val2;
}
