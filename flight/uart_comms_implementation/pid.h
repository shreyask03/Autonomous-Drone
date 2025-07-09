#ifndef PID_H
#define PID_H
#include <Arduino.h>


class PID{
  public:
    // used to send gains in a neat package
    struct Gains{
      float p;
      float i;
      float d;
    };
  private:
    float K_p;
    float K_i;
    float K_d;

    float i_sum = 0;
    float prev_err = 0;
    float prev_measured = 0;
    float output_limit;
    float i_limit;
    unsigned long last_update = 0;
    
  public:
    PID(float P, float I, float D, float limit = 250);
    float compute(float setpoint, float measured);
    void reset();
    Gains getTune();
    void setTune(float p = -1, float i = -1, float d = -1);
};

#endif