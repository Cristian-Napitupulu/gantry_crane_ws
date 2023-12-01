#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "microROS.h"

void limitSwitchCallback()
{
  if (limitSwitchEncoderSide.getState() == LOW)
  {
    limitSwitchEncoderSideState = true;
  }

  if (limitSwitchTrolleyMotorSide.getState() == LOW)
  {
    limitSwitchTrolleyMotorSideState = true;
  }
}

bool reverse = false; 

void setup()
{
  ledBuiltIn.begin();
  ledBuiltIn.turnOn();

  trolleyMotor.begin();
  hoistMotor.begin();

  encoderTrolley.begin();
  attachInterrupt(encoderTrolley.getChannelA(), encoderTrolleyCallback, CHANGE);

  limitSwitchEncoderSide.begin();
  attachInterrupt(limitSwitchEncoderSide.getPin(), limitSwitchCallback, CHANGE);

  limitSwitchTrolleyMotorSide.begin();
  attachInterrupt(limitSwitchTrolleyMotorSide.getPin(), limitSwitchCallback, CHANGE);

  microROSInit();

  ledBuiltIn.turnOff();
}

void loop()
{
  ledBuiltIn.pulse();

  RCSOFTCHECK(rclc_executor_spin_some(&limitSwitchExecutor, RCL_MS_TO_NS(LIMIT_SWITCH_PUBLISH_TIMEOUT_NS)));
  RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_NS)));
  RCSOFTCHECK(rclc_executor_spin_some(&motorPWMExecutor, RCL_MS_TO_NS(MOTOR_PWM_SUBSCRIBER_TIMEOUT_NS)));
}