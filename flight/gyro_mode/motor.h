#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor{
  private:
    uint8_t pin;
  public:
    Motor(uint8_t motorPin); // motor object constructor
    void init(); // sets pin to output
    void write(int pulse); // sends pwm pulse to esc, input is pulse in microseconds
};


#endif