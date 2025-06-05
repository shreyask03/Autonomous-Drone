#include "pid.h"

PID::PID(float P, float I, float D, float limit = 250) : K_p(P), K_i(I), K_d(D), output_limit(limit), i_limit(limit / 2.0f) {}


float PID::compute(float setpoint, float measured){
  float error = setpoint - measured;
  unsigned long now = millis();
  float dt = (now - last_update) / 1000.0f; // delta t in seconds
  last_update = now;

  if (dt <= 0.0f || dt > 1.0f) dt = 0.01f; // prevent extreme spikes

  // integral term

  i_sum += error*dt;
  i_sum = constrain(i_sum,-i_limit,i_limit); // prevent integral windup by constraining to max i value

  // derivative term
  float derivative = (error - prev_err) / dt;
  prev_err = error;

  // PID output
  float output = this->K_p*error + this->K_i * i_sum + this->K_d * derivative;
  return constrain(output, -output_limit, output_limit);

}

void PID::reset(){
  i_sum = 0;
  prev_err = 0;
}

