#ifndef PID_H
#define PID_H
#include <Arduino.h>


class PID{
  private:
    float K_p;
    float K_i;
    float K_d;

    float i_sum = 0;
    float prev_err = 0;
    float output_limit;
    float i_limit;
    unsigned long last_update = 0;
  public:
    PID(float P, float I, float D, float limit = 250);
    float compute(float setpoint, float measured);
    void reset();
};

#endif