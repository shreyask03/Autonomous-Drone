#include "mpu.h"

MPU mpu;

unsigned long last_upd = 0;
float prev_pitch = 0;
float prev_roll = 0;

void setup() {
  // put your setup code here, to run once:
  mpu.init();

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.updateGyro();
  mpu.updateAccel();

  MPU::Vector3 gyro = mpu.getGyro();
  MPU::Vector3 accel = mpu.getAccel();

  float gx = gyro.x;
  float gy = gyro.y;

  float ax = accel.x;
  float ay = accel.y;
  float az = accel.z;

  // accel angle estimate
  float accel_pitch = atan2(ax,sqrt(ay*ay + az*az)) * 180.0/PI;
  float accel_roll = atan2(ay,sqrt(ax*ax + az*az)) * 180.0/PI;

  // gyro angle estimate
  unsigned long now = millis();
  float dt = (now - last_upd)/1000.0f;
  last_upd = now;
  float dpitch = gy*dt;
  float gyro_pitch = prev_pitch + dpitch;
  prev_pitch = gyro_pitch;

  float droll = gx*dt;
  float gyro_roll = prev_roll + droll;
  prev_roll = gyro_roll;

  // complementary filter
  float alpha = 0.98; // weighting factor

  float filtered_pitch = alpha*gyro_pitch + (1 - alpha)*accel_pitch;
  float filtered_roll = alpha*gyro_roll + (1 - alpha)*accel_roll;

  //COMMENT OUT: DEBUG USE ONLY
  // Serial.print("Accel Pitch: "); Serial.print(accel_pitch); Serial.print(" | ");
  // Serial.print("Accel Roll: "); Serial.print(accel_roll); Serial.print(" | ");
  // Serial.print("Gyro Pitch: "); Serial.print(gyro_pitch); Serial.print(" | ");
  // Serial.print("Gyro Roll: "); Serial.println(gyro_roll);
  Serial.print("Filtered Pitch: ");Serial.print(filtered_pitch); Serial.print(" | ");
  Serial.print("Filtered Roll: ");Serial.println(filtered_roll);


}
