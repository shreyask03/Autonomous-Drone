#ifndef MPU_H
#define MPU_H
#include <Arduino.h>
#include "Wire.h"

class MPU{
  private:
    float gx = 0.0; // raw gyro data for each axis LSB/Deg/s, where LSB is least significant bit
    float gy = 0.0;
    float gz = 0.0;
    static constexpr float GY250_SENSE = 131.0f; // at +- 250 deg/s, the sensitivity scale factor is 131 LSB/deg/s -> a reading of 1 deg/s will output as 131 in serial monitor by default
    static const int MPU_ADDR = 0x68;
    static const int PWR_MGMT1 = 0x6B;

  public:
    void init(); // initialize MPU
    void readRawData();
    float convertYaw(); // convert to usable rates
    float convertPitch();
    float convertRoll();

};

#endif