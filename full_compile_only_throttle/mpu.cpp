#include "mpu.h"


void MPU::init(){
  // initialize MPU
  Wire.begin();
  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(PWR_MGMT1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void MPU::readRawData(){
  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(0x43); // starting address GYRO_X high byte
  Wire.endTransmission(false); // restart for reading

  Wire.requestFrom(this->MPU_ADDR,6); // get 16 bytes for the 3 axes in gyro data sent as 2 8 byte packets each
  int16_t rawGx = (Wire.read() << 8) | Wire.read(); // gyro x high byte logic OR with low byte to combine into 1 raw data value
  int16_t rawGy = (Wire.read() << 8) | Wire.read(); // gyro y ""
  int16_t rawGz = (Wire.read() << 8) | Wire.read(); // gyro z ""
  this->gx = rawGx;
  this->gy = rawGy;
  this->gz = rawGz;
}

float MPU::convertYaw(){
  // convert to usable deg/s from raw LSB/deg/s
  return this->gz / this->GY250_SENSE;
}

float MPU::convertPitch(){
  // convert to usable deg/s from raw LSB/deg/s
  return this->gy / this->GY250_SENSE;
}

float MPU::convertRoll(){
  // convert to usable deg/s from raw LSB/deg/s
  return this->gx / this->GY250_SENSE;
}


