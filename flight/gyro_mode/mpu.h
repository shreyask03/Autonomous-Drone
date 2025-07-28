

// #ifndef MPU_H
// #define MPU_H

// #include <Arduino.h>
// #include "Wire.h"
// // #include "MPU6050_6Axis_MotionApps20.h"
// // #include "I2CDev.h"


// class MPU{
//   public:
//     // Quick access to 3 axis data such as angles or rates both inside and outside mpu class
//     struct Vector3{
//       float x = 0.0;
//       float y = 0.0;
//       float z = 0.0; 
//     };
//   private:
//      // Provides access to DMP functionality provided by ElectronicCats MPU6050 Library
//     // MPU6050 mpu;
//     // Will contain angular rates for all YPR (yaw,pitch,roll)
//     Vector3 gyro;

//     // Will contain linear accelerations for YPR
//     Vector3 accel;

//     // contains calculated angles (yaw will be 0, not calculated)
//     Vector3 angles;

//     // At +- 250 deg/s, the sensitivity scale factor is 131 LSB/deg/s according to the MPU-6050 datasheet -> a reading of 1 deg/s will output as 131 in serial monitor by default
//     static constexpr float GY250_SENSE = 131.0f;
//     // At +- 4g, the sensitivity scale factor is 8192 LSB/g according to the MPU-6050 datasheet
//     static constexpr float ACCEL4G_SENSE = 8192.0f;
//     // I2C device address
//     static const int MPU_ADDR = 0x68;
//     // I2C register address for config. wake mode
//     static const int PWR_MGMT1 = 0x6B;

//     /*for filtering*/
//     float alpha = 0.9;
//     // gyro
//     float new_gx = 0;
//     float new_gy = 0;
//     float new_gz = 0;
//     float lastTime = 0;
//     float prev_filtered_roll = 0;
//     float prev_filtered_pitch = 0;
//     bool firstRun = true;
//     // accel
//     float new_ax = 0;
//     float new_ay = 0;
//     float new_az = 0;

//   public:
//     // Initialize MPU
//     void init();
//     // Read raw data then conduct post processing to get angular rates in the correct units [deg/s]
//     void updateGyro();

//     // Read raw data then conduct post processing to get linear accelerations in correct units [g's]
//     void updateAccel();
//     // Return rates
//     Vector3 getGyro() const;
//     /* -------------------------- ANGLE MODE USE ------------------------ */
//     // onboard angle calculation
//     Vector3 computeAngles();

//     // debugging tool
//     void printData();
//     // Initialize DMP onboard MPU
//     // void setupForDMP();
//     // Return angles calculated through DMP
//     // Vector3 getDMPAngles();

//     //wait for DMP to fully initialize to avoid sending garbage angles
//     // void waitForStableDMP();
// };

// #endif



#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include "Wire.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "I2CDev.h"


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
    // MPU6050 mpu;
    // Will contain angular rates for all YPR (yaw,pitch,roll)
    Vector3 gyro;

    // Will contain linear accelerations for YPR
    Vector3 accel;

    // contains calculated angles (yaw will be 0, not calculated)
    Vector3 angles;

    // for applying offsets to eliminate drift
    Vector3 gyroOffset;
    Vector3 accelOffset;

    // At +- 250 deg/s, the sensitivity scale factor is 131 LSB/deg/s according to the MPU-6050 datasheet -> a reading of 1 deg/s will output as 131 in serial monitor by default
    static constexpr float GY250_SENSE = 131.0f;
    // At +- 4g, the sensitivity scale factor is 8192 LSB/g according to the MPU-6050 datasheet
    static constexpr float ACCEL4G_SENSE = 8192.0f;
    // I2C device address
    static const int MPU_ADDR = 0x68;
    // I2C register address for config. wake mode
    static const int PWR_MGMT1 = 0x6B;

    /*for filtering*/
    const float alpha = 0.98f;
    const float gyro_alpha = 0.9f;
    // gyro
    float new_gx = 0;
    float new_gy = 0;
    float new_gz = 0;
    float lastTime = 0;
    float prev_filtered_roll = 0;
    float prev_filtered_pitch = 0;
    bool firstRun = true;
    // accel
    float new_ax = 0;
    float new_ay = 0;
    float new_az = 0;

  public:
    // Initialize MPU
    void init();
    // Read raw data then conduct post processing to get angular rates in the correct units [deg/s]
    void updateGyro();

    // Read raw data then conduct post processing to get linear accelerations in correct units [g's]
    void updateAccel();
    // Return rates
    Vector3 getGyro() ;
    Vector3 getAccel() ;
    /* -------------------------- ANGLE MODE USE ------------------------ */
    // onboard angle calculation
    Vector3 computeAngles();

    // debugging tool
    void printData();

    void calibrate(uint16_t samples = 12000);
    Vector3 getRawGyro() const;
    Vector3 getRawAccel() const;
};

#endif