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

  // apply filtering to smooth gyro noise
  new_gx = alpha*gyro.x + (1-alpha)*new_gx;
  new_gy = alpha*gyro.y + (1-alpha)*new_gy;
  new_gz = alpha*gyro.z + (1-alpha)*new_gz;

  // update vector with filtered values, account for calibrated offsets
  gyro.x = new_gx - 0.31;
  gyro.y = new_gy - 0.32;
  gyro.z = new_gz - 1.05;

}

MPU::Vector3 MPU::getGyro() { // getter function
  return gyro;
}

MPU::Vector3 MPU::getAccel(){
  return accel;
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
  new_ax = alpha*accel.x + (1-alpha)*new_ax;
  new_ay = alpha*accel.y + (1-alpha)*new_ay;
  new_az = alpha*accel.z + (1-alpha)*new_az;

  // update vector with filtered values, account for calibrated offsets
  accel.x = new_ax - 0.35;
  accel.y = new_ay - -0.01;
  accel.z = new_az - 1.11;
}

MPU::Vector3 MPU::computeAngles(){
  float startTime = micros();
  float dt = (startTime - lastTime)/1000000.0f;

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
  }
  float gyro_roll_angle = prev_filtered_roll + gyro.x*dt;
  float gyro_pitch_angle = prev_filtered_pitch + gyro.y*dt;

  // combine through complimentary filter
  float filtered_roll = alpha*gyro_roll_angle + (1-alpha)*accel_roll_angle;
  float filtered_pitch = alpha*gyro_pitch_angle + (1-alpha)*accel_pitch_angle;

  // update prev angles
  prev_filtered_roll = filtered_roll;
  prev_filtered_pitch = filtered_pitch;

  // // Adjust for static mounting bias (degrees)
  // const float pitchOffset = -9.3f; // measured bias (sign depends on direction)
  // const float rollOffset = 0.1f;  // adjust if needed

  // // Apply correction
  // filtered_pitch -= pitchOffset;
  // filtered_roll  -= rollOffset;

  // add to angle vector
  angles.x = filtered_roll;
  angles.y = filtered_pitch;
  
  lastTime = startTime;
  
  return angles;
}

void MPU::calibrate(uint16_t samples=12000){
  Vector3 gyroSum;
  Vector3 accelSum;

  Serial.println("Starting IMU calibration. Keep device level and still... ");

  for(uint16_t i = 0; i < samples; i++){
    updateGyro();
    updateAccel();

    Vector3 g = getRawGyro();
    Vector3 a = getRawAccel();

    gyroSum.x += g.x;
    gyroSum.y += g.y;
    gyroSum.z += g.z;
    accelSum.x += a.x;
    accelSum.y += a.y;
    accelSum.z += a.z;

    delay(2); // 500hz sampling rate
  }

  // compute average offset
  gyroOffset.x = gyroSum.x / samples;
  gyroOffset.y = gyroSum.y / samples;
  gyroOffset.z = gyroSum.z / samples;

  accelOffset.x = accelSum.x / samples;
  accelOffset.y = accelSum.y / samples;
  accelOffset.z = (accelSum.z / samples) - 1.0f; // subtract gravity

  Serial.println("Calibration complete. Offsets:");
  Serial.print("Gyro offset: ");
  Serial.print(gyroOffset.x); Serial.print(", ");
  Serial.print(gyroOffset.y); Serial.print(", ");
  Serial.println(gyroOffset.z);

  Serial.print("Accel offset: ");
  Serial.print(accelOffset.x); Serial.print(", ");
  Serial.print(accelOffset.y); Serial.print(", ");
  Serial.println(accelOffset.z);
}
