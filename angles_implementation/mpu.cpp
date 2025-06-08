#include "mpu.h"


void MPU::init(){
  // initialize MPU

  // Wake up MPU-6050 (disable sleep mode)
  Wire.begin();
  delay(100);

  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(PWR_MGMT1);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);

  // configure accelerometer (+- 4g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x08); // Full scale range +- 4g
  Wire.endTransmission(true);

  // configure gyroscope (+- 250 deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00); // +- 250 deg/s
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
  gyro.x = static_cast<float>(rawGx) / this->GY250_SENSE;
  gyro.y = static_cast<float>(rawGy) / this->GY250_SENSE;
  gyro.z = static_cast<float>(rawGz) / this->GY250_SENSE;
}

void MPU::updateAccel(){
  Wire.beginTransmission(this->MPU_ADDR);
  Wire.write(0x3B); // starting address ACCEL_X high byte
  Wire.endTransmission(false); // restart for reading

  Wire.requestFrom(this->MPU_ADDR, 6); // get 16 bytes for 3 axes in accel data sent as 2 8 byte packets each
  int16_t rawAx = (Wire.read() << 8) | Wire.read(); // accel x high byte logic OR with low byte to combine into 1 raw data value
  int16_t rawAy = (Wire.read() << 8) | Wire.read(); // accel y ""
  int16_t rawAz = (Wire.read() << 8) | Wire.read(); // accel z ""

  accel.x = static_cast<float>(rawAx) / this->ACCEL_SCALE;
  accel.y = static_cast<float>(rawAy) / this->ACCEL_SCALE;
  accel.z = static_cast<float>(rawAz) / this->ACCEL_SCALE;


  // COMMENT OUT: DEBUG USE ONLY
  // Serial.print("Raw Ax: ");Serial.print(rawAx); Serial.print(" | ");
  // Serial.print("Raw Ay: ");Serial.print(rawAy); Serial.print(" | ");
  // Serial.print("Raw Az: ");Serial.print(rawAz); Serial.print(" | ");
  // Serial.print("Ax (g): ");Serial.print(accel.x);Serial.print(" | ");
  // Serial.print("Ay (g): ");Serial.print(accel.y);Serial.print(" | ");
  // Serial.print("Az (g): ");Serial.println(accel.z);


}

MPU::Vector3 MPU::getGyro() const{
  return gyro;
}

MPU::Vector3 MPU::getAccel() const{
  return accel;
}