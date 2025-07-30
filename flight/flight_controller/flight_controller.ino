#include "receiver.h"
#include "motor.h"
#include "mpu.h"
#include "pid.h"
#include "comms.h"

//initialize sensors and motors
Receiver rx;

Motor FL(11); // front left
Motor FR(3); // front right
Motor BL(10); // back left
Motor BR(9); // back right

MPU mpu;

static constexpr int RATE_LIMIT = 150; // deg/s
static constexpr int ANGLE_LIMIT = 30; // deg

// initialize PID axes for rate control
PID roll(0.6,1.5,0.08);
PID pitch(0.75,1.5,0.08);
PID yaw(1,1.5,0.005);

// initialize new PID objects for angle control
PID rollAngle(2.75,0.2,0.2,RATE_LIMIT);
PID pitchAngle(3.25,0.2,0.2,RATE_LIMIT);

// initialize communication object
Comms comms(&pitch,&roll,&yaw,&pitchAngle,&rollAngle,&rx);

//initialize control loop variables
int throttle;

float desiredRoll;
float desiredPitch;
float desiredYaw;

float desiredRollRate;
float desiredPitchRate;

float currRollRate;
float currPitchRate;
float currYawRate;

float rollContribution;
float pitchContribution;
float yawContribution;

// Global variables for loop timing purposes
unsigned long prev_acro_loop = 0;
unsigned long prev_angle_loop = 0;
const unsigned long ACRO_PERIOD = 4000; // microsecond value locks 250 hz loop rate
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

  // initialize mpu
  mpu.init();

  // reset PID values
  roll.reset();
  pitch.reset();
  yaw.reset();
  pitchAngle.reset();
  rollAngle.reset();
  comms.init();
  
}

void loop() {

  unsigned long now = micros(); // loop timer variable

  if(!rx.isArmed()){
    // shut motors off
    FL.write(1000); // 1ms pulse = 0% throttle
    FR.write(1000);
    BL.write(1000);
    BR.write(1000);
    // reset PID values
    roll.reset();
    pitch.reset();
    yaw.reset();
    pitchAngle.reset();
    rollAngle.reset();

    rx.waitForConnect();

    static String serialInput = "";
    char buffer[100];  // <-- declare buffer here

    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        serialInput.trim();
        if (serialInput.length() > 0) {
          serialInput.toCharArray(buffer, sizeof(buffer));
          comms.parseMessage(buffer);
        }
        serialInput = "";
      } else {
        serialInput += c;
      }
    }
  }
  else{

    /* ANGLE MODE - 100 hz
        ACRO MODE - 250 hz*/

    // ANGLE MODE
    if(rx.isAngle()){

      // calculate desired rates at 100hz 
      if(now - prev_angle_loop >= ANGLE_PERIOD){
        prev_angle_loop += ANGLE_PERIOD;

        // get pilot inputs
        throttle = rx.getThrottle();
        desiredRoll = rx.getAngle(1,ANGLE_LIMIT);
        desiredPitch = rx.getAngle(2,ANGLE_LIMIT);
        desiredYaw = rx.getRate(4, RATE_LIMIT);

        mpu.updateGyro();
        mpu.updateAccel();
        MPU::Vector3 angles = mpu.computeAngles();

        float currRollAngle = angles.x;
        float currPitchAngle = angles.y;

        MPU::Vector3 rates = mpu.getGyro();
        currYawRate = rates.z;

        // compute desired rates from angle PID loops
        desiredRollRate = rollAngle.compute(desiredRoll, currRollAngle);
        desiredPitchRate = pitchAngle.compute(desiredPitch, currPitchAngle);
        // desired yaw is handled in rate mode

      }

      // inner rate loop at 250hz
      if(now - prev_acro_loop >= ACRO_PERIOD){
        prev_acro_loop += ACRO_PERIOD;

        throttle = rx.getThrottle(); // keep throttle updated at 250hz

        if(throttle >= 1100){
          // read gyro rates
          mpu.updateGyro();
          MPU::Vector3 rates = mpu.getGyro();

          currRollRate = rates.x;
          currPitchRate = rates.y;
          currYawRate = rates.z;

          // compute PID outputs for each axis
          rollContribution = roll.compute(desiredRollRate,currRollRate);
          pitchContribution = pitch.compute(desiredPitchRate,currPitchRate);
          yawContribution = yaw.compute(desiredYaw,currYawRate);
        } else{
          rollContribution = 0;
          pitchContribution = 0;
          yawContribution = 0;
          roll.reset();
          pitch.reset();
          yaw.reset();
          rollAngle.reset();
          pitchAngle.reset();
        }

        // mix motor channels (i.e. add PID corrections to throttle)
        int base_throttle = throttle; // copy throttle to issues with overwriting or tearing with value being read in

        // Sign convention: pitch forward(front down) is positive, roll right(right down) is positive, yaw right(clockwise viewed from top) is positive
        int pulse_FL = constrain(base_throttle - pitchContribution + rollContribution - yawContribution,1000,2000);
        int pulse_FR = constrain(base_throttle - pitchContribution - rollContribution + yawContribution,1000,2000);
        int pulse_BL = constrain(base_throttle + pitchContribution + rollContribution + yawContribution,1000,2000);
        int pulse_BR = constrain(base_throttle + pitchContribution - rollContribution - yawContribution,1000,2000);


        // write to motors
        FL.write(pulse_FL);
        FR.write(pulse_FR);
        BL.write(pulse_BL);
        BR.write(pulse_BR);

      }

    }

    // ACRO MODE (RATE LOOP ONLY)
    else{

      if(now - prev_acro_loop >= ACRO_PERIOD){
        prev_acro_loop += ACRO_PERIOD;

        // get pilot inputs
        throttle = rx.getThrottle();
        desiredRoll = rx.getRate(1, RATE_LIMIT);
        desiredPitch = rx.getRate(2, RATE_LIMIT);
        desiredYaw = rx.getRate(4, RATE_LIMIT);

        if(throttle >= 1100){
          // get current rates
          mpu.updateGyro();
          MPU::Vector3 rates = mpu.getGyro();

          currRollRate = rates.x;
          currPitchRate = rates.y;
          currYawRate = rates.z;

          // calculate PID contributions
          rollContribution = roll.compute(desiredRoll,currRollRate);
          pitchContribution = pitch.compute(desiredPitch,currPitchRate);
          yawContribution = yaw.compute(desiredYaw, currYawRate);

        } else{
          rollContribution = 0;
          pitchContribution = 0;
          yawContribution = 0;
          roll.reset();
          pitch.reset();
          yaw.reset();
        }

        // mix motor channels (i.e. add PID corrections to throttle)
        int base_throttle = throttle; // copy throttle to issues with overwriting or tearing with value being read in

        // Sign convention: pitch forward(front down) is positive, roll right(right down) is positive, yaw right(clockwise viewed from top) is positive
        int pulse_FL = constrain(base_throttle - pitchContribution + rollContribution - yawContribution,1000,2000);
        int pulse_FR = constrain(base_throttle - pitchContribution - rollContribution + yawContribution,1000,2000);
        int pulse_BL = constrain(base_throttle + pitchContribution + rollContribution + yawContribution,1000,2000);
        int pulse_BR = constrain(base_throttle + pitchContribution - rollContribution - yawContribution,1000,2000);


        // write to motors
        FL.write(pulse_FL);
        FR.write(pulse_FR);
        BL.write(pulse_BL);
        BR.write(pulse_BR);
        
      }
    }
  }
}
