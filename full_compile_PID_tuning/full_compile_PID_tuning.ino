#include "receiver.h"
#include "motor.h"
#include "mpu.h"
#include "pid.h"

//initialize sensors and motors
Receiver rx;

Motor FL(11); // front left
Motor FR(3); // front right
Motor BL(10); // back left
Motor BR(9); // back right

MPU mpu;

// initialize PID axes
PID roll(0.5,0.5,0.0001); // try same as pitch values
PID pitch(0.5,0.5,0.0001); // P=0.75, D = 0.0001 pretty good // P=0.5 slightly better // overall relatively sluggish response, may fix later wih post flight tuning
PID yaw(0.5,0.5,0.0001);

//initialize control loop variables
int throttle = 0;
int desiredRoll = 0;
int desiredPitch = 0;
int desiredYaw = 0;

float currRollRate = 0.0;
float currPitchRate = 0.0;
float currYawRate = 0.0;

float rollContribution = 0.0;
float pitchContribution = 0.0;
float yawContribution = 0.0;


ISR(PCINT0_vect){
  rx.handleInterrupt();
}

void setup() {
  // put your setup code here, to run once:
  // debugging use
  // Serial.begin(115200);

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
  }
  else{ 
    // Read pilot inputs (desired "setpoint")
    throttle = rx.getThrottle();
    desiredRoll = rx.getRollRate();
    desiredPitch = rx.getPitchRate();
    desiredYaw = rx.getYawRate();

    if(throttle >= 1100){ // if throttle low dont add PID corrections to output to avoid injury when handling
    // Read IMU (current orientation)
    mpu.readRawData();
    currRollRate = mpu.convertRoll();
    currPitchRate = mpu.convertPitch();
    currYawRate = mpu.convertYaw();

    // compute PID correction
    rollContribution = roll.compute(desiredRoll, currRollRate );
    pitchContribution = pitch.compute(desiredPitch, currPitchRate );
    yawContribution = yaw.compute(desiredYaw, currYawRate );
    }
    else{
      rollContribution = 0;
      pitchContribution = 0;
      yawContribution = 0;
    }

    // mix motor channels (i.e. add PID corrections to throttle)
    int base_throttle = throttle; // copy throttle to issues with overwriting or tearing with value being read in


    // Sign convention: pitch forward(front down) is positive, roll right(right down) is positive, yaw right(clockwise viewed from top) is positive
    int pulse_FL = base_throttle - pitchContribution + rollContribution - yawContribution;
    int pulse_FR = base_throttle - pitchContribution - rollContribution + yawContribution;
    int pulse_BL = base_throttle + pitchContribution + rollContribution + yawContribution;
    int pulse_BR = base_throttle + pitchContribution - rollContribution - yawContribution;

    // write to motors
    FL.write(pulse_FL);
    FR.write(pulse_FR);
    BL.write(pulse_BL);
    BR.write(pulse_BR);

    // // debugging test prints
    // Serial.print("pitchCont - "); Serial.print(pitchContribution);
  }
}
