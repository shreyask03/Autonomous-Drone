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
int throttle;
int desiredRoll;
int desiredPitch;
int desiredYaw;

float currRollRate;
float currPitchRate;
float currYawRate;

float rollContribution;
float pitchContribution;
float yawContribution;

// Global variables for loop timing purposes
unsigned long prev_acro_loop = 0;
unsigned long prev_angle_loop = 0;
const unsigned long ACRO_PERIOD = 4000; // microsecond value locks 250 hz loop rate (acro mode)
const unsigned long ANGLE_PERIOD = 10000; // microsecond value locks 100 hz loop rate (angle mode)


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

  // initialize gyro and dmp
  mpu.init();
  mpu.setupForDMP();

  // reset PID values
  roll.reset();
  pitch.reset();
  yaw.reset();

  
}

void loop() {
  if(!rx.isArmed()){
    // shut motors off
    FL.write(1000); // 1ms pulse = 0% throttle
    FR.write(1000);
    BL.write(1000);
    BR.write(1000);
    rx.waitForConnect();
    
  }
  else{
    unsigned long now = micros();
    if(rx.isAngle() && (now - prev_angle_loop >= ANGLE_PERIOD)){
      prev_angle_loop += ANGLE_PERIOD;
      throttle = rx.getThrottle();
      if(throttle >= 1100){
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
      }
      else{
        rollContribution = 0;
        pitchContribution = 0;
        yawContribution = 0;
      }
    }
    else if(!rx.isAngle() && (now - prev_acro_loop >= ACRO_PERIOD)){
      prev_acro_loop += ACRO_PERIOD;
      // Read pilot inputs (desired "setpoint") mapped to rates
      throttle = rx.getThrottle();
      desiredRoll = rx.getRollRate();
      desiredPitch = rx.getPitchRate();
      desiredYaw = rx.getYawRate();

      if(throttle >= 1100){ // if throttle low dont add PID corrections to output to avoid injury when handling
        // Read IMU (current orientation)
        mpu.updateGyro();
        MPU::Vector3 rates = mpu.getGyro();
        currRollRate = rates.x;
        currPitchRate = rates.y;
        currYawRate = rates.z;

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
      // rx.printSignals(); // COMMENT OUT: DEBUGGING USE ONLY

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

    }
  }
}
