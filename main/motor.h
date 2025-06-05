#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

class Motor{
  private:
    uint8_t pin;
    uint16_t minPulse; // microseconds
    uint16_t maxPulse;
    uint16_t throttleUs; // desired throttle in microseconds

  public:
    Motor(uint8_t motorPin, uint16_t minPulseUs = 1000, uint16_t maxPulseUs = 2000);
    void init(); // set up timers to generate 488 Hz PWM 
    void setPulse(uint16_t us); // direct microsecond pulse
    void writePulse(); // send one PWM pulse (you call this in loop every 4ms)
};

#endif