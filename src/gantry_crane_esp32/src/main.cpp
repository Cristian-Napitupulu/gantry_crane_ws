#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.h"

void checkLimitSwitch();

void findOrigin();

void moveToMiddle();

void setup()
{
  ledBuiltIn.begin();
  ledBuiltIn.turnOn();

  trolleyMotor.begin();
  hoistMotor.begin();

  encoderTrolley.begin();
  attachInterrupt(encoderTrolley.getChannelA(), encoderTrolleyCallback, CHANGE);

  limitSwitchEncoderSide.begin();

  limitSwitchTrolleyMotorSide.begin();

  ledBuiltIn.turnOff();

  microROSInit();

  Wire.begin();

  if (!analogToDigitalConverter.init())
  {
    Serial.println("ADS1115 not connected!");
  }

  findOrigin();
}

void loop()
{
  ledBuiltIn.pulse();

  checkLimitSwitch();

  // moveToMiddle();

  RCSOFTCHECK(rclc_executor_spin_some(&limitSwitchExecutor, RCL_MS_TO_NS(LIMIT_SWITCH_PUBLISH_TIMEOUT_NS)));
  RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_NS)));
  RCSOFTCHECK(rclc_executor_spin_some(&motorPWMExecutor, RCL_MS_TO_NS(MOTOR_PWM_SUBSCRIBER_TIMEOUT_NS)));
}

void checkLimitSwitch()
{
  if (limitSwitchEncoderSide.getState() == LOW)
  {
    limitSwitchEncoderSideState = true;
    trolleyMotor.setPWM(0);
    trolleyMotor.brake();
  }
  else
  {
    limitSwitchEncoderSideState = false;
  }

  if (limitSwitchTrolleyMotorSide.getState() == LOW)
  {
    limitSwitchTrolleyMotorSideState = true;
    trolleyMotor.setPWM(0);
    trolleyMotor.brake();
  }
  else
  {
    limitSwitchTrolleyMotorSideState = false;
  }
}

void findOrigin()
{
  int counter = 0;
  while (counter < 2)
  {
    // Serial.println("Finding origin");
    ledBuiltIn.blink(100);
    if (limitSwitchEncoderSide.getState() != LOW)
    {
      trolleyMotor.setPWM(-90);
    }
    else
    {
      trolleyMotor.setPWM(0);
      counter++;
      delay(500);
    }
    RCSOFTCHECK(rclc_executor_spin_some(&limitSwitchExecutor, RCL_MS_TO_NS(LIMIT_SWITCH_PUBLISH_TIMEOUT_NS)));
    RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_NS)));
  }
  delay(1000);
  counter = 0;
  while (counter < 2)
  {
    ledBuiltIn.blink(100);
    for (int i = -60; i > -120; i--)
    {
      trolleyMotor.setPWM(i);
      delay(10);
    }
    encoderTrolley.reset();
    trolleyMotor.setPWM(0);
    counter++;
    delay(500);
    RCSOFTCHECK(rclc_executor_spin_some(&limitSwitchExecutor, RCL_MS_TO_NS(LIMIT_SWITCH_PUBLISH_TIMEOUT_NS)));
    RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_NS)));
  }
}

void moveToMiddle()
{
  int32_t error = (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) / 2 - encoderTrolley.getPulse();
  int16_t pwm = 255 * error / ((ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) / 2);
  trolleyMotor.setPWM(pwm);
}