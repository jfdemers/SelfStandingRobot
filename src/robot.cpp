#include "robot.h"

#include "display.h"
#include "motors.h"
#include "mpu.h"

#define DISPLAY_SPEED
//#define CALCTIME

float power = 0;
float increment = 0.01;

#define K_p 0.15f   // 15 original
#define K_i 1.0f  // 100 original
#define K_d 0.0f   // 0.15 original
#define MaxAngleOffset 5.5f
#define wheelRateGain 0.003f
#define wheelPosGain 0.0015f

#define DT 5000 // 5 msec time step
#define FHz 200 // Sample Rate

Robot::Robot() {
  // Set an initial value of 10V. It will change rapidly once we
  // start taking measurement.
  voltage = 11;
  voltageTimer = 0;
  voltageTimerOut = 0;
  MotorScaleFactor = 1.0f;
  stopped = false;
  Error = 0;
  IntState = 0;
  PosCmd = 0;
  rotationCmd = 0;
  PitchEst = 0;
  BiasEst = 0;
  TorqueCMD = 0;
  TurnTorque = 0;
}

void Robot::setup() {
  Serial.begin(SerialSpeed);
  // Wait before sending commands to the display.
  delay(100);
  Display::setDisplay(Display::no_cursor);

  motors.initialize();
  motors.stop();
  motors.standby();

  mpu.initialize();

  Display::clear();
  Display::setPos(2, 0);
  Serial.print("Lay robot down");
  Display::setPos(0, 1);
  Serial.print("and press button");
  pinMode(LED_BUILTIN, OUTPUT);

  waitForButton();

  Display::clear();
  Display::setPos(0, 0);
  Serial.print("Calibrating gyro");

  while (mpu.calibrateYGyro()) {

  }

  standUp();

  motors.resume();

  Display::clear();
  Display::setPos(0,0);
  Serial.print("Voltage:");

  lastTime = micros();
}

void Robot::run() {
  if (stopped) {
    return;
  }

  float AngleOffset;

  motors.update();

  // Read and filter gyro data
  mpu.readIMUData();
  kalmanFilter(mpu.getAccAngle(), mpu.getGyroRate());

  if (millis() - voltageTimer > 1000) {
    if (!checkVoltage()) {
      stopAll();
    }
    voltageTimer = millis();
  }

  #ifndef CALCTIME
  if (millis() - voltageTimerOut > 10000) {
    Display::setPos(9, 1);
    Serial.print(voltage, 1); Serial.print("V");
    voltageTimerOut = millis();
  }
  #endif

  if (PitchEst < -45 || PitchEst > 45) {  // Robot has fallen down
    motors.stop();
    motors.reset();

    // Reset integrator and encoder position
    IntState = 0;

    PosCmd = 0; rotationCmd = 0;

    standUp();  // Ask user to stand robot back up
    return;
  }
  else {

    // Filter ThrottleIn and convert to AOCmd
    /*if (ThrottleIn > 800 && ThrottleIn < 2200 && !RechargeBattery) {  // Valid range
      if ((abs(ThrottleIn - ThrottleInGood) < 200) || (throttle_glitch_persistent > 20)) { // Changes greater than 200 are assumed to be a glitch
        ThrottleInGood = ThrottleIn;
        throttle_glitch_persistent = 0;
      }
      else {
        throttle_glitch_persistent++;
      }
      ThrottleF = (float)ThrottleInGood;
      AOCmd = (ThrottleF - 1500.0f) / 75.0f; // SteeringIn ranges from 1000 to 2000.  Rescale to -6.7 to +6.7 deg
    }
    else {
      throttle_glitch_persistent++;
      if (throttle_glitch_persistent > 20) {
        throttle_glitch_persistent = 0;
      }
      AOCmd = 0;
    }*/
    AOCmd = 0;

    AngleOffset = constrain(AOCmd, -MaxAngleOffset, MaxAngleOffset);  // Limits throttle input

    if (abs(AOCmd) > 1.0f) { // If angle offset command greater than 1 deg, then commanding forward/reverse motion
       PosCmd = motors.getWheelPosition() + 1.0f * motors.getVelocity();  // Set encoder position command to a location ahead of robot,
                                                      // allowing robot to drift to a new location when stopping
       AngleOffset -=  motors.getVelocity() * wheelRateGain;
    }
    else { // Stop robot
      // Apply encoder feedback outer loop
      AngleOffset -= (motors.getWheelPosition() - PosCmd) * wheelPosGain  + motors.getVelocity() * wheelRateGain;  // PD controller
    }
    AngleOffset = constrain(AngleOffset, -MaxAngleOffset, MaxAngleOffset);  // Additional limiter after outer loop

    Error = AngleOffset - PitchEst;
    //Error = 0 - PitchEst;
    IntState = IntState + Error / FHz;
    IntState = constrain(IntState, -5.0f, 5.0f);

    TorqueCMD = K_p * Error + K_i * IntState - K_d * mpu.getGyroRate();  // PID Feedback Control

    // Filter SteeringIn and convert to TurnTorque
    /*if (SteeringIn > 800 && SteeringIn < 2200 && !RechargeBattery) {  // Valid range
      if ((abs(SteeringIn - SteeringInGood) < 200) || (steer_glitch_persistent > 20)) { // Changes greater than 200 are assumed to be a glitch
        SteeringInGood = SteeringIn;
        steer_glitch_persistent = 0;
      }
      else {
        steer_glitch_persistent++;
      }
      SteeringF = SteeringInGood;
      TurnTorque = (SteeringF - 1500.0f) / 20.0f;  // SteeringIn ranges from 1000 to 2000.  Rescale to -25 to 25.
      TurnTorque = constrain(TurnTorque, -25.0f, 25.0f);
      if (abs(TurnTorque) < 5.0f && abs(AOCmd) < 2.0f) {
        TurnTorque += Kp_Rotation * (rotationCmd - rotationAngle);  // Maintain current rotation angle
      }
      else {
        rotationCmd = rotationAngle;
      }
    }
    else {
      steer_glitch_persistent++;
      if (steer_glitch_persistent > 20) {
        steer_glitch_persistent = 0;
      }
      TurnTorque = 0;
    }*/

    TurnTorque = 0;
  }

  motors.setPower((TorqueCMD - TurnTorque), (TorqueCMD + TurnTorque));
  /*static float direction = 1.0f;
  static unsigned long dirTime = 0;
  dirTime += 1;

  if (dirTime > 20) {
    if (direction <= 0) {
      direction = 1.0f;
    }
    else if (direction >= 1.0) {
      direction = -1.0f;
    }
    //direction = direction * -1.0f;
    dirTime = 0;
  }
  motors.setPower(1.0f * direction, 1.0f * direction);*/

  #ifdef CALCTIME
  static unsigned long CalcTime, MaxCalcTime, AveCalcTime, TimeCounter, NumSamples;
  CalcTime = micros() - lastTime;
  if (MaxCalcTime < CalcTime) {
    MaxCalcTime = CalcTime;
  }
  AveCalcTime += CalcTime;
  NumSamples++;
  if ((millis() - TimeCounter) > 1000) {
    TimeCounter = millis();
    AveCalcTime /= NumSamples;
    Display::clear();
    Serial.print("Ave: ");
    Serial.print((float)AveCalcTime / 1000.0f);
    Display::setPos(0, 1);
    Serial.print("Max: ");
    Serial.print((float)MaxCalcTime / 1000.0f);
    Display::setPos(8, 1);
    Serial.print(NumSamples);
    NumSamples = 0;
    MaxCalcTime = 0;
    AveCalcTime = 0;
  }
  #endif

  while (micros() - lastTime < DT) {

  }
  lastTime = micros();
}

bool Robot::checkVoltage() {
  // Filter the voltage value in case we have a bad reading. If the value
  // is ok, it will soon be reflected in the voltage value.
  float currentValue = analogRead(VoltSensorPin) * voltageUnit;
  voltage = (voltage * 4 + currentValue) / 5;

  MotorScaleFactor = 11.8f / voltage;

  if (voltage < 10) {
    return false;
  }

  return true;
}

void Robot::stopAll() {
  stopped = true;
  motors.stop();

  Display::clear();
  Display::setPos(0, 0);
  Serial.print("Error");
}

void Robot::waitForButton() {
  // This is not the most efficient way to read the value of the pin, we could
  // just take the digital value, but it's not recommended to take digital
  // readings when we use the analog function on the chip because it can cause
  // invalid readings.
  while (analogRead(PushButtonPin) < 800) {
  }
  while (analogRead(PushButtonPin) > 600) {

  }
}

void Robot::standUp() {
  Display::clear();
  Display::setPos(0, 0);
  Serial.print("Stand & Push Btn");

  int button_pushed = 0;
  unsigned long timer;
  while (!button_pushed) {  // Calculate and display pitch until button pushed
    timer = millis();
    PitchEst = 0;
    for (int i = 0; i < 50; i++) {  // Average 50 samples
      mpu.readIMUData();
      PitchEst += mpu.getAccAngle();
      if (analogRead(PushButtonPin) > 800) {
        button_pushed = 1;
      }
    }
    PitchEst /= 50.0f;

    Display::setPos(0, 1);
    Serial.print("Pitch: ");
    Serial.print(PitchEst);
    while (millis() < timer + 250) {
      if (analogRead(PushButtonPin) > 800) {
        button_pushed = 1;
      }
    }
  }
  BiasEst = 0;
}

  void Robot::kalmanFilter(float pitchMeas, float rateMeas) {
    /*
     * Steady State Kalman Filter -- Hard code gains into function
     *
     * [pitch; bias]_(k+1) = A * [pitch; bias]_(k) + B * rate_meas + G * w
     * pitch = C * [pitch; bias] + v
     *
     * A = [1 -dt; 0 1];
     * B = [dt; 0];
     * C = [1 0];
     *
     * G = [1 0; 0 1];
     * cov(w) = Q;
     * cov(v) = R;
     *
     * Kalman Filter gain solved using dlqe in Octave (GNU version of Matlab), assuming:
     * Q = diag([0.001 0.001]);
     * R = 1.5;
     * dt = 0.01;
     * then: K = dlqe(A,G,C,Q,R) = [0.033810; -0.025380];
     */
      const float dt = 0.005;
      const float K[2] = {0.016905, -0.012690}; // Gains originally calculated for 100 Hz,
      // Divide by 2 for 200 Hz gains
      float y = pitchMeas - PitchEst;
      PitchEst += dt * (rateMeas - BiasEst);
      PitchEst += K[0] * y;
      BiasEst  += K[1] * y;

      //PitchEst = pitchMeas;
  }
