#include "mpu.h"

void MPU::init(){
  // initialize MPU
  Wire.begin();
  Wire.beginTransmission(this->MPU_ADDR); // begin writing to MPU
  Wire.write(PWR_MGMT1); // access register
  Wire.write(0); // wake MPU
  Wire.endTransmission(true);

  // /* RUN FOR THE FIRST TIME ONCE THEN COMMENT OUT */
  // // configure gyro
  // Wire.beginTransmission(this->MPU_ADDR);
  // Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  // Wire.write(0x00);                   // set register at 0x00 (+- 250 deg/s)
  // Wire.endTransmission(true);
  // delay(20);

  // // configure accel
  // Wire.beginTransmission(this->MPU_ADDR);
  // Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  // Wire.write(0x08);                  //set register at 0x08 (+- 4g)
  // Wire.endTransmission(true);

}

void MPU::updateGyro(){
  Wire.beginTransmission(this->MPU_ADDR); // begin writing to MPU
  Wire.write(0x43); // starting address GYRO_X high byte register
  Wire.endTransmission(false); // restart transmission for reading

  Wire.requestFrom(this->MPU_ADDR,6); // get 16 bytes for the 3 axes in gyro data sent as two 8 byte packets each
  int16_t rawGx = (Wire.read() << 8) | Wire.read(); // gyro x high byte logic OR with low byte to combine into 1 raw data value
  int16_t rawGy = (Wire.read() << 8) | Wire.read(); // gyro y ""
  int16_t rawGz = (Wire.read() << 8) | Wire.read(); // gyro z ""


  gyro.x = rawGx / this->GY250_SENSE; // divide by scale factor to get to deg/s, store in vector
  gyro.y = rawGy / this->GY250_SENSE;
  gyro.z = rawGz / this->GY250_SENSE;
<<<<<<< Updated upstream
=======

  // apply filtering to smooth gyro noise
  gyro.x = alpha*gyro.x + (1-alpha)*new_gx;
  new_gx = gyro.x;
  gyro.y = alpha*gyro.y + (1-alpha)*new_gy;
  new_gy = gyro.y;
  gyro.z = alpha*gyro.z + (1-alpha)*new_gz;
  new_gz = gyro.z;

  // update vector with filtered values
  gyro.x = new_gx;
  gyro.y = new_gy;
  gyro.z = new_gz;
>>>>>>> Stashed changes
}

MPU::Vector3 MPU::getGyro() const{ // getter function
  return gyro;
}

void MPU::updateAccel(){
  Wire.beginTransmission(this->MPU_ADDR); // begin writing to MPU
  Wire.write(0x3B); // starting address ACCEL_X_H high byte register
  Wire.endTransmission(false); // restart transmission for reading

  Wire.requestFrom(this->MPU_ADDR,6); // get 16 bytes for 3 axes in accel data sent as two 8 byte packets each
  int16_t rawAx = (Wire.read() << 8) | Wire.read(); // accel x high byte logic OR with low byte to combine into 1 16 bit raw data value
  int16_t rawAy = (Wire.read() << 8) | Wire.read(); // accel y 
  int16_t rawAz = (Wire.read() << 8) | Wire.read(); // accel z


  // convert raw data to proper units
  accel.x = rawAx / this->ACCEL4G_SENSE;
  accel.y = rawAy / this->ACCEL4G_SENSE;
  accel.z = rawAz / this->ACCEL4G_SENSE;

  // apply filtering to smooth accel noise
  accel.x = alpha*accel.x + (1-alpha)*new_ax;
  new_ax = accel.x;
  accel.y = alpha*accel.y + (1-alpha)*new_ay;
  new_ay = accel.y;
  accel.z = alpha*accel.z + (1-alpha)*new_az;
  new_az = accel.z;

  // update vector with filtered values
  accel.x = new_ax;
  accel.y = new_ay;
  accel.z = new_az;
}

MPU::Vector3 MPU::computeAngles(){
  float startTime = micros();
  float dt = (startTime - lastTime)/1000000.0f;

<<<<<<< Updated upstream
    // convert angles from radians to degrees
    // sign convention (roll right (right wing down) = +, pitch forward (nose down) = +, yaw right (nose right) = +)
    angles.x = ypr[2] * 180/M_PI; //  roll
    angles.y = -ypr[1] * 180/M_PI; // pitch
    angles.z = ypr[0] * 180/M_PI; // yaw
=======
  // safety clamping dt to avoid weird spikes
  if(dt < 0.001f) dt = 0.001f;
  if(dt > 0.02f) dt = 0.02f;
  
  // compute accel angle
  float accel_roll_angle = atan2(accel.y, sqrt(accel.x*accel.x + accel.z*accel.z)) *57.2957; // deg
  float accel_pitch_angle = atan2(-1*accel.x, sqrt(accel.y*accel.y + accel.z*accel.z)) * 57.2957; // deg 

  // compute gyro angle
  // initial angle should not be hard set to 0 (in case drone starts tilted), so setting to current accel initially will be good enough
  if(firstRun){
    prev_filtered_roll = accel_roll_angle;
    prev_filtered_pitch = accel_pitch_angle;
    firstRun = false;
>>>>>>> Stashed changes
  }
  float gyro_roll_angle = prev_filtered_roll + gyro.x*dt;
  float gyro_pitch_angle = prev_filtered_pitch + gyro.y*dt;

  // combine through complimentary filter
  float filtered_roll = alpha*gyro_roll_angle + (1-alpha)*accel_roll_angle;
  float filtered_pitch = alpha*gyro_pitch_angle + (1-alpha)*accel_pitch_angle;

  // update prev angles
  prev_filtered_roll = filtered_roll;
  prev_filtered_pitch = filtered_pitch;

  // add to angle vector
  angles.x = filtered_roll;
  angles.y = filtered_pitch;
  
  lastTime = startTime;
  
  return angles;
<<<<<<< Updated upstream
}
=======
}
>>>>>>> Stashed changes
