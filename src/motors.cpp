#include "motors.h"

//#define USE_ANALOG_WRITE

#include <Wire.h>

#ifndef USE_ANALOG_WRITE
  #include "PWM.h"              // PWM Frequency Library at https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads
#endif
#include "EnableInterrupt.h"  // Enable Interrupt library
#include "digitalWriteFast.h" // DigitalWriteFast Library
#include "display.h"

#define PWM_FREQUENCY   10000

#define WHEEL_DIAMETER  78.0f
#define WHEELS_DISTANCE 173.0f
#define TICKS_PER_TURN  90.0f
#define DEG_PER_TICK    (360.0f / TICKS_PER_TURN)

#define REVERSE_RIGHT_ENCODER

//#define LEFT_ENCODER_FLAG   0x01
//#define RIGHT_ENCODER_FLAG  0x02

static volatile long LeftEncoderTicks = 0;
static volatile long RightEncoderTicks = 0;
//static volatile uint8_t flags = 0;

static volatile uint8_t LeftEncoderBValue = 0;
static volatile uint8_t RightEncoderBValue = 0;

static void handleLeftEncoderInt() {
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBValue = digitalReadFast(LeftEncoderPinB);   // read the input pin

  // and adjust counter + if A leads B
  LeftEncoderTicks += LeftEncoderBValue ? -1 : +1;
  //flags |= LEFT_ENCODER_FLAG;
}

static void handleRightEncoderInt() {
  RightEncoderBValue = digitalReadFast(RightEncoderPinB);

  #ifdef REVERSE_RIGHT_ENCODER
  RightEncoderTicks += RightEncoderBValue ? 1 : -1;
  #else
  RightEncoderTicks += RightEncoderBValue ? -1 : 1;
  #endif
  //flags |= RIGHT_ENCODER_FLAG;
}

Motors::Motors() {
  powerFactor = 1.0;
  wheelPosition = 0;
}

void Motors::initialize() {
  // Setup the motor power pins
  pinMode(LeftMotorIn1, OUTPUT);
  pinMode(LeftMotorIn2, OUTPUT);
  pinMode(RightMotorIn1, OUTPUT);
  pinMode(RightMotorIn2, OUTPUT);
  pinMode(StandbyPin, OUTPUT);

  digitalWriteFast(LeftMotorIn1, LOW);
  digitalWriteFast(LeftMotorIn2, LOW);
  digitalWriteFast(RightMotorIn1, LOW);
  digitalWriteFast(RightMotorIn2, LOW);
  digitalWriteFast(StandbyPin, HIGH);

  // Setup the encoders pins
  pinMode(LeftEncoderPinA, INPUT);
  pinMode(LeftEncoderPinB, INPUT);
  pinMode(RightEncoderPinA, INPUT);
  pinMode(RightEncoderPinB, INPUT);

  // Active pullups resistors for encoder pins.
  digitalWrite(LeftEncoderPinA, HIGH);
  digitalWrite(LeftEncoderPinB, HIGH);
  digitalWrite(RightEncoderPinA, HIGH);
  digitalWrite(RightEncoderPinB, HIGH);

  // Setup the motors PWM pins
  #ifndef USE_ANALOG_WRITE
  InitTimersSafe(PWM_FREQUENCY);
  SetPinFrequency(LeftMotorPWM, PWM_FREQUENCY);
  SetPinFrequency(RightMotorPWM,  PWM_FREQUENCY);
  #else
  pinMode(LeftMotorPWM, OUTPUT);
  pinMode(RightMotorPWM, OUTPUT);
  #endif

  // Enable interrupts for the encoders
  enableInterrupt(LeftEncoderPinA, handleLeftEncoderInt, RISING);
  enableInterrupt(RightEncoderPinA, handleRightEncoderInt, RISING);

  lastTime = millis();
}

void Motors::setPowerFactor(float value) {
  powerFactor = value;
}

void Motors::setPower(float p1, float p2) {
  if (p1 < 0.0f) {
    digitalWriteFast(LeftMotorIn1, LOW);
    digitalWriteFast(LeftMotorIn2, HIGH);
  }
  else {
    digitalWriteFast(LeftMotorIn1, HIGH);
    digitalWriteFast(LeftMotorIn2, LOW);
  }

  if (p2 < 0.0f) {
    digitalWriteFast(RightMotorIn1, LOW);
    digitalWriteFast(RightMotorIn2, HIGH);
  }
  else {
    digitalWriteFast(RightMotorIn1, HIGH);
    digitalWriteFast(RightMotorIn2, LOW);
  }

  #ifndef USE_ANALOG_WRITE
  unsigned int realP1 = 65535 * fmin(fabs(p1 * powerFactor), 1.0f);
  unsigned int realP2 = 65535 * fmin(fabs(p2 * powerFactor), 1.0f);

  pwmWriteHR(LeftMotorPWM, realP1);
  pwmWriteHR(RightMotorPWM, realP2);
  #else
  unsigned int realP1 = 255 * fmin(fabs(p1 * powerFactor), 1.0f);
  unsigned int realP2 = 255 * fmin(fabs(p2 * powerFactor), 1.0f);

  analogWrite(LeftMotorPWM, realP1);
  analogWrite(RightMotorPWM, realP2);
  #endif
}

void Motors::standby() {
  digitalWriteFast(StandbyPin, LOW);
}

void Motors::resume() {
  digitalWriteFast(StandbyPin, HIGH);
}

void Motors::stop() {
  digitalWriteFast(LeftMotorIn1, LOW);
  digitalWriteFast(LeftMotorIn2, LOW);
  digitalWriteFast(RightMotorIn1, LOW);
  digitalWriteFast(RightMotorIn2, LOW);

  #ifndef USE_ANALOG_WRITE
  pwmWriteHR(LeftMotorPWM, 0);
  pwmWriteHR(RightMotorPWM, 0);
  #else
  analogWrite(LeftMotorPWM, 0);
  analogWrite(RightMotorPWM, 0);
  #endif
}

void Motors::update() {
  long currentTime = millis();

  if (currentTime - lastTime >= 100) {
    lastTime = currentTime;

    // Disable interrupts while we get the values of the encoder.
    noInterrupts();

    leftEncoderTicks = LeftEncoderTicks;
    rightEncoderTicks = RightEncoderTicks;

    interrupts();

    double lastWheelPosition = wheelPosition;
    wheelPosition = 0.5 * (leftEncoderTicks + rightEncoderTicks) * DEG_PER_TICK; //0.5 * (leftEncoderTicks + rightEncoderTicks) * DEG_PER_TICK;
    velocity = 10.0f * (wheelPosition - lastWheelPosition);
    rotation = 0.5f * (leftEncoderTicks - rightEncoderTicks) * DEG_PER_TICK * (WHEEL_DIAMETER / WHEELS_DISTANCE);
  }
}

void Motors::reset() {
  lastTime = millis();
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
  velocity = 0;
  rotation = 0;
  wheelPosition = 0;
}
