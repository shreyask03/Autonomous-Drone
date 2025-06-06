  #ifndef MPU_H
#define MPU_H
#include <Arduino.h>
#include "Wire.h"

class MPU{
  public:
    struct Vector3{
      float x = 0.0;
      float y = 0.0;
      float z = 0.0; 
    };
  private:

    Vector3 gyro; // contains angular rates for all 3 axes
    Vector3 accel; // contains linear accels for all 3 axes

    static constexpr float GY250_SENSE = 131.0f; // at +- 250 deg/s, the sensitivity scale factor is 131 LSB/deg/s -> a reading of 1 deg/s will output as 131 in serial monitor by default
    static constexpr float ACCEL_SCALE = 16384.0f; // sensitivity scale factor
    static const int MPU_ADDR = 0x68;
    static const int PWR_MGMT1 = 0x6B;

  public:
    void init(); // initialize MPU
    void updateGyro(); // read raw data then conduct post processing to get angular rates in the correct units [deg/s]
    void updateAccel(); // read raw data then conduct post processing to get linear accelerations in the correct units [g's]

    // getter functions
    Vector3 getGyro() const;
    Vector3 getAccel() const;
};

#endif