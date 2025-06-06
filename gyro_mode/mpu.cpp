#include "mpu.h"


void MPU::init(){
  // initialize MPU
  Wire.begin();
  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(PWR_MGMT1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void MPU::updateGyro(){
  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(0x43); // starting address GYRO_X high byte
  Wire.endTransmission(false); // restart for reading

  Wire.requestFrom(this->MPU_ADDR,6); // get 16 bytes for the 3 axes in gyro data sent as 2 8 byte packets each
  int16_t rawGx = (Wire.read() << 8) | Wire.read(); // gyro x high byte logic OR with low byte to combine into 1 raw data value
  int16_t rawGy = (Wire.read() << 8) | Wire.read(); // gyro y ""
  int16_t rawGz = (Wire.read() << 8) | Wire.read(); // gyro z ""
  gyro.x = rawGx / this->GY250_SENSE;
  gyro.y = rawGy / this->GY250_SENSE;
  gyro.z = rawGz / this->GY250_SENSE;
}

void MPU::updateAccel(){
  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(0x3B); // starting address ACCEL_X high byte
  Wire.endTransmission(false); // restart for reading

  Wire.requestFrom(this->MPU_ADDR, 6); // get 16 bytes for 3 axes in accel data sent as 2 8 byte packets each
  int16_t rawAx = (Wire.read() << 8) | Wire.read(); // accel x high byte logic OR with low byte to combine into 1 raw data value
  int16_t rawAy = (Wire.read() << 8) | Wire.read(); // accel y ""
  int16_t rawAz = (Wire.read() << 8) | Wire.read(); // accel z ""
  accel.x = rawAx / this->ACCEL_SCALE;
  accel.y = rawAy / this->ACCEL_SCALE;
  accel.z = rawAz / this->ACCEL_SCALE;
}

MPU::Vector3 MPU::getGyro() const{
  return gyro;
}

MPU::Vector3 MPU::getAccel() const{
  return accel;
}