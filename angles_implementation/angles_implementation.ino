#include "mpu.h"

MPU mpu;

unsigned long prev_loop = 0;
const unsigned long LOOP_PERIOD = 4000; // microsecond value locks 250 hz loop rate
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mpu.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long last_angle_time_upd = micros(); // track to get angle calculation on right timing
  unsigned long now = micros();
  if(now - prev_loop >= LOOP_PERIOD){
    prev_loop += LOOP_PERIOD; // prevents loop drift over time reducing refresh rate losses

    float dt = (now - last_angle_time_upd)/ 1e6;
    last_angle_time_upd = now;
    MPU::Vector3 angles = mpu.computeAngles(dt);
  float x = angles.x;
  float y = angles.y;
  // float z = angles.z;

  //COMMENT OUT: DEBUG USE ONLY
  // Serial.print("Accel Pitch: "); Serial.print(accel_pitch); Serial.print(" | ");
  // Serial.print("Accel Roll: "); Serial.print(accel_roll); Serial.print(" | ");
  // Serial.print("Gyro Pitch: "); Serial.print(gyro_pitch); Serial.print(" | ");
  // Serial.print("Gyro Roll: "); Serial.println(gyro_roll);
  Serial.print("X ");Serial.print(x); Serial.print(" | ");
  Serial.print("Y ");Serial.print(y); Serial.println(" | ");
  // Serial.print("Z ");Serial.println(z);

  }


}
