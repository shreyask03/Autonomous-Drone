#include "pid.h"

PID::PID(float P, float I, float D, float limit = 250) : K_p(P), K_i(I), K_d(D), output_limit(limit), i_limit(limit / 2.0f) {}


float PID::compute(float setpoint, float measured){
  float error = setpoint - measured;
  unsigned long now = micros();
  float dt = (now - last_update) / 1000000.0f; // delta t in seconds
  last_update = now;

 // prevent extreme spikes from dt calculation glitches (near zero division or negligible integral help)
 if(dt < 0.0005f) dt = 0.0005f;
 if(dt > 0.02f && dt < 0.5f) dt=0.02f;
 if(dt >= 0.5f){ // missed update, glitch
    this->reset(); //clear integral and derivative state
    return 0.0f; // don't compute until system recovers
 }

  // integral term

  i_sum += error*dt;
  i_sum = constrain(i_sum,-i_limit,i_limit); // prevent integral windup by constraining to max i value
  // classical derivative term, works on error, can cause derivative kick for quick setpoint changes
  // float derivative = (error - prev_err) / dt;
  // prev_err = error;
  
  // measurement based derivative term, works with measured value and generally avoids kick issues (if this is active, the derivative term must be subtracted from output)
  // float derivative = -1 * (measured - prev_measured) / dt;
  // prev_measured = measured;

  // === Derivative term (measured value, filtered and rate-limited) ===
  float raw_derivative = -1 * (measured - prev_measured) / dt;
  prev_measured = measured;

  // Optional: constrain raw derivative to reduce spike potential
  raw_derivative = constrain(raw_derivative, -d_limit, d_limit);

  // Low-pass filter derivative (exponential smoothing)
  float derivative = d_filter_alpha * prev_derivative + (1 - d_filter_alpha) * raw_derivative;
  prev_derivative = derivative;

  // PID output
  float output = this->K_p*error + this->K_i * i_sum + this->K_d * derivative;
  return constrain(output, -output_limit, output_limit);

}

void PID::reset(){
  i_sum = 0;
  prev_err = 0;

  prev_measured = 0;
  prev_derivative = 0;
  last_update = micros();
}

PID::Gains PID::getTune(){
  return Gains{K_p,K_i,K_d};

}

void PID::setTune(float p = -1, float i = -1, float d = -1){
  if(p >= 0) this->K_p = p;
  if(i >= 0) this->K_i = i;
  if(d >= 0) this->K_d = d;
}

>>>>>>> Stashed changes
