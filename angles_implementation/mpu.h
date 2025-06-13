

#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2CDev.h"


class MPU{
  public:
    // Quick access to 3 axis data such as angles or rates both inside and outside mpu class
    struct Vector3{
      float x = 0.0;
      float y = 0.0;
      float z = 0.0; 
    };
  private:
     // Provides access to DMP functionality provided by ElectronicCats MPU6050 Library
    MPU6050 mpu;
    // Will contain angular rates for all YPR (yaw,pitch,roll)
    Vector3 gyro;
    // At +- 250 deg/s, the sensitivity scale factor is 131 LSB/deg/s -> a reading of 1 deg/s will output as 131 in serial monitor by default
    static constexpr float GY250_SENSE = 131.0f;
     // I2C device address
    static const int MPU_ADDR = 0x68;
    // I2C register address for config. wake mode
    static const int PWR_MGMT1 = 0x6B;
  public:
    // Initialize MPU
    void init();
    // Read raw data then conduct post processing to get angular rates in the correct units [deg/s]
    void updateGyro();
    // Return rates
    Vector3 getGyro() const;
    /* -------------------------- ANGLE MODE USE ------------------------ */
    // Initialize DMP onboard MPU
    void setupForDMP();
    // Return angles calculated through DMP
    Vector3 getDMPAngles();
    
};

#endif