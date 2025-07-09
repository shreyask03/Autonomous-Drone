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

static constexpr int RATE_LIMIT = 60; // deg/s
static constexpr int ANGLE_LIMIT = 30; // deg
// initialize PID axes for rate control
// X class white pla symmetric working values -> P 0.5, I 0.5, D 0.0001
// X class green PLA asymmetric -> 
PID roll(1,1.5,0.005);
PID pitch(1,1.5,0.005);
PID yaw(1,1.5,0.005);

// initialize new PID objects for angle control
PID rollAngle(0.1,0,0,RATE_LIMIT);
PID pitchAngle(0,0,0,RATE_LIMIT); // P = 0.8 , I = 0.01, D = 0.015

// initialize communication object
Comms comms(&pitch,&roll,&yaw,&pitchAngle,&rollAngle,&rx);

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
const unsigned long ACRO_PERIOD = 4000; // microsecond value locks 250 hz loop rate
const unsigned long ANGLE_PERIOD = 4000; // microsecond value locks 250 hz loop rate (angle mode)
float dt = 0;


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
  // if (Serial.availableForWrite()) {
  //   delay(100); // Wait a moment for Serial1 to connect
  //   comms.sendTune(); // Send the tune once to Portenta
  // }
  
}

void loop() {
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

    // if (Serial.available()) {
    //   static char buffer[100];
    //   size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    //   buffer[len] = '\0';
    //   comms.parseMessage(buffer);
    // }
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
    unsigned long now = micros();
    if(rx.isAngle() && (now - prev_angle_loop >= ANGLE_PERIOD)){
      prev_angle_loop += ANGLE_PERIOD;


      // calculate desired angle and angular rate setpoints from pilot
      throttle = rx.getThrottle();
      desiredRoll = rx.getAngle(1, ANGLE_LIMIT);
      desiredPitch = rx.getAngle(2, ANGLE_LIMIT);
      desiredYaw = rx.getRate(4, RATE_LIMIT); // in angle mode, yaw remains in "rate" mode

      if(throttle >= 1100){
        // get current orientation from IMU

        // for roll and pitch angle controller
        mpu.updateGyro();
        mpu.updateAccel();
        MPU::Vector3 angles = mpu.computeAngles();
        // for inner rate controller
        MPU::Vector3 rates = mpu.getGyro();
        
        float currRollAngle = angles.x;
        float currPitchAngle = angles.y;

        float currRollRate = rates.x;
        float currPitchRate = rates.y;
        float currYawRate = rates.z;

        // PID corrections
        // outer loop for angle control
        float desiredRollRate = rollAngle.compute(desiredRoll, currRollAngle);
        float desiredPitchRate = pitchAngle.compute(desiredPitch, currPitchAngle);
        
        // inner loop for nested rate control
        rollContribution = roll.compute(desiredRollRate,currRollRate);
        pitchContribution = pitch.compute(desiredPitchRate,currPitchRate);
        yawContribution = yaw.compute(desiredYaw,currYawRate);
      }
      else{
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
    else if(!rx.isAngle() && (now - prev_acro_loop >= ACRO_PERIOD)){
      prev_acro_loop += ACRO_PERIOD;

      // Read pilot inputs (desired "setpoint") mapped to rates
      throttle = rx.getThrottle();
      desiredRoll = rx.getRate(1,90);
      desiredPitch = rx.getRate(1,90);
      desiredYaw = rx.getRate(1,90);

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
