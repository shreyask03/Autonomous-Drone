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
    float prev_derivative = 0; // for derivative smoothing
    float d_limit = 300.0f; // Optional: Max deg/s change rate
    float d_filter_alpha = 0.6f; // Smoothing factor for D
    unsigned long last_update = 0;
  public:
    PID(float P, float I, float D, float limit = 250);
    float compute(float setpoint, float measured);
    void reset();
    Gains getTune();
    void setTune(float p = -1, float i = -1, float d = -1);
};

#endif