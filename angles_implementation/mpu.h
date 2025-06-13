

#ifndef MPU_H
#define MPU_H
#include <Arduino.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2CDev.h"


class MPU{
  public:
    struct Vector3{
      float x = 0.0;
      float y = 0.0;
      float z = 0.0; 
    };
  private:

    MPU6050 mpu; // for use with DMP

    Vector3 gyro; // contains angular rates for all 3 axes
    Vector3 accel; // contains linear accels for all 3 axes
    Vector3 gyro_bias;

    static constexpr float GY250_SENSE = 131.0f; // at +- 250 deg/s, the sensitivity scale factor is 131 LSB/deg/s -> a reading of 1 deg/s will output as 131 in serial monitor by default
    static constexpr float ACCEL_SCALE = 16384.0f; // sensitivity scale factor
    static const int MPU_ADDR = 0x68;
    static const int PWR_MGMT1 = 0x6B;

    // important variables used in computeAngles() for gyroscope angle integration
    unsigned long last_upd = 0;
    float prev_pitch = 0;
    float prev_roll = 0;

  public:
    void init(); // initialize MPU
    void updateGyro(); // read raw data then conduct post processing to get angular rates in the correct units [deg/s]
    void updateAccel(); // read raw data then conduct post processing to get linear accelerations in the correct units [g's]

    // getter functions
    Vector3 getGyro() const;
    Vector3 getAccel() const;

    // Angle Mode
    void setupForDMP();
    Vector3 computeAngles(float dt); // calculates and applies complementary filter to accel and gyro data
    void calibrateGyro(int samples = 500); // corrects for drift
    Vector3 getDMPAngles();
    
};

#endif