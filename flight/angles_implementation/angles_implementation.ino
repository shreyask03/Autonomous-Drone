#include "mpu.h"

MPU mpu;

// Global variables for loop timing purposes
unsigned long prev_acro_loop = 0;
unsigned long prev_angle_loop = 0;
const unsigned long ACRO_PERIOD = 4000; // microsecond value locks 250 hz loop rate (acro mode)
const unsigned long ANGLE_PERIOD = 10000; // microsecond value locks 100 hz loop rate (angle mode)

// TEMPORARY: Testing angle calculation functionality | Delete and adjust dependent code when merged with main
bool angleMode = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mpu.init();

  // mpu.calibrate();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long now = micros();
  /* -------------------------- ANGLE MODE -------------------------- */
  if(angleMode && (now - prev_angle_loop >= ANGLE_PERIOD)){
    prev_angle_loop += ANGLE_PERIOD;// prevents loop drift over time reducing refresh rate losses


    /*sign convention (roll right (right wing down) = +, pitch forward (nose down) = +, yaw right (nose right) = +)*/
    mpu.updateGyro();
    mpu.updateAccel();
    MPU::Vector3 angles = mpu.computeAngles();
    float x = angles.x;
    float y = angles.y;
    float z = angles.z;

    /*COMMENT OUT: DEBUG USE ONLY | angle calculation debug print*/
    Serial.print("roll ");Serial.print(x); Serial.print(" | ");
    Serial.print("pitch ");Serial.println(y); //Serial.print(" | ");
    // Serial.print("yaw ");Serial.println(z);
    // delay(30);


  }

  /* -------------------------- ACRO MODE ------------------------- */
  else if(!angleMode && (now - prev_acro_loop >= ACRO_PERIOD)){
    prev_acro_loop += ACRO_PERIOD; // prevents loop drift over time reducing refresh rate losses
    mpu.updateGyro();
    MPU::Vector3 rates = mpu.getGyro();
    float rollrate = rates.x;
    float pitchrate = rates.y;
    float yawrate = rates.z;

    /*COMMENT OUT: DEBUG USE ONLY | angular rate calculation debug print*/
    Serial.print("roll_rate = ");Serial.print(rollrate);
    Serial.print(" | pitch_rate = ");Serial.print(pitchrate);
    Serial.print(" | yaw_rate = ");Serial.println(yawrate);
  }


}
