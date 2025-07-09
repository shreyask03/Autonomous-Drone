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
  i_sum = constrain(i_sum,-1*i_limit,i_limit); // prevent integral windup by constraining to max i value

  // classical derivative term, works on error, can cause derivative kick for quick setpoint changes
  float derivative = (error - prev_err) / dt;
  prev_err = error;
  
  // measurement based derivative term, works with measured value and generally avoids kick issues
  // float derivative = (measured - prev_measured) / dt;
  // prev_measured = measured;

  // PID output
  float output = this->K_p*error + this->K_i * i_sum + this->K_d * derivative;
  return constrain(output, -1*output_limit, output_limit);

}

void PID::reset(){
  i_sum = 0;
  prev_err = 0;
}

PID::Gains PID::getTune(){
  return Gains{K_p,K_i,K_d};

}

void PID::setTune(float p = -1, float i = -1, float d = -1){
  if(p >= 0) this->K_p = p;
  if(i >= 0) this->K_i = i;
  if(d >= 0) this->K_d = d;
}

