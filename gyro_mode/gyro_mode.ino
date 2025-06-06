#include "receiver.h"
#include "motor.h"
#include "mpu.h"
#include "pid.h"

Receiver rx;

Motor FL(11); // front left
Motor FR(3); // front right
Motor BL(10); // back left
Motor BR(9); // back right

MPU mpu;

PID roll(0.0,0.0,0.0); // initial untuned values P 0.1, I 0, D 0.01 // next D 0.5
PID pitch(0.0,0.0,0.0); // initial untuned values P 0.1, I 0, D 0.01 // next D 0.5
PID yaw(0.0,0.0,0.0);

ISR(PCINT0_vect){
  rx.handleInterrupt();
}

void setup() {
  // put your setup code here, to run once:
  // initialize receiver
  rx.init();

  // wait for rx and tx to establish connection
  rx.waitForConnect();

  // now safe to initialize motors
  FL.init();
  FR.init();
  BL.init();
  BR.init();

  // initialize gyro
  mpu.init();

  // reset PID values
  roll.reset();
  pitch.reset();
  yaw.reset();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!rx.isArmed()){
    // shut motors off
    FL.write(1000); // 1ms pulse = 0% throttle
    FR.write(1000);
    BL.write(1000);
    BR.write(1000);
    rx.waitForConnect();

    // rx.printSignals(); // COMMENT OUT: DEBUGGING USE ONLY
  }
  else{
    if(rx.isAngle()){ // ANGLE MODE
      
    }
    else{ // GYRO MODE
      // Read IMU (current orientation)

      mpu.readRawData();
      float currRollRate = mpu.convertRoll();
      float currPitchRate = mpu.convertPitch();
      float currYawRate = mpu.convertYaw();

      // Read pilot inputs (desired "setpoint")
      int throttle = rx.getThrottle();
      // int desiredRoll = rx.getRollRate();
      // int desiredPitch = rx.getPitchRate();
      // int desiredYaw = rx.getYawRate();
      int desiredRoll, desiredPitch, desiredYaw;
      mpu.convertGyroRawToUsable(desiredRoll, desiredPitch, desiredYaw);

      // compute PID correction
      float rollContribution = roll.compute(desiredRoll, currRollRate );
      float pitchContribution = pitch.compute(desiredPitch, currPitchRate );
      float yawContribution = yaw.compute(desiredYaw, currYawRate );

      // mix motor channels (i.e. add PID corrections to throttle)
      int base_throttle = throttle; // copy throttle to issues with overwriting or tearing with value being read in

      int pulse_FL = base_throttle + pitchContribution - rollContribution - yawContribution;
      int pulse_FR = base_throttle + pitchContribution + rollContribution + yawContribution;
      int pulse_BL = base_throttle - pitchContribution - rollContribution + yawContribution;
      int pulse_BR = base_throttle - pitchContribution + rollContribution - yawContribution;

      // write to motors
      FL.write(pulse_FL);
      FR.write(pulse_FR);
      BL.write(pulse_BL);
      BR.write(pulse_BR);
    }
  }
}
