#ifndef _ROBOT_H
#define _ROBOT_H

#include "config.h"

#include "motors.h"
#include "mpu.h"

class Robot {
public:
  Robot();

  void setup();
  void run();

private:
  //void initializeMPU();
  bool checkVoltage();
  void stopAll();

  void waitForButton();
  void standUp();

  void kalmanFilter(float pitchMeas, float rateMeas);

  bool stopped;

  float voltage;

  double PitchEst;
  double BiasEst;

  unsigned long lastTime;
  long voltageTimer;
  long voltageTimerOut;

  float AOCmd;
  double PosCmd;
  float Error;
  double IntState;
  float rotationCmd;

  float TorqueCMD;
  float TurnTorque;

  float MotorScaleFactor;

  Motors motors;
  MPU mpu;
};

#endif
