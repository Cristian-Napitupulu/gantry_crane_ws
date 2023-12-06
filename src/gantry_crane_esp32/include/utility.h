#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>
#include "variable.h"

void splitInt32toInt16(int32_t inputInt32, int16_t &lowerInt16, int16_t &upperInt16)
{
  // Extract the lower 16 bits (first int16)
  lowerInt16 = static_cast<int16_t>(inputInt32 & 0xFFFF);

  // Right shift the input by 16 bits to get the upper 16 bits
  upperInt16 = static_cast<int16_t>((inputInt32 >> 16) & 0xFFFF);
}

float map_value(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void checkLimitSwitch()
{
  if (limitSwitchEncoderSide.getState() == LOW)
  {
    limitSwitchEncoderSideState = true;
    if (trolleyMotorPWM < 0)
    {
      trolleyMotorPWM = 0;
    }
  }
  else
  {
    limitSwitchEncoderSideState = false;
  }

  if (limitSwitchTrolleyMotorSide.getState() == LOW)
  {
    limitSwitchTrolleyMotorSideState = true;
    if (trolleyMotorPWM > 0)
    {
      trolleyMotorPWM = 0;
    }
  }
  else
  {
    limitSwitchTrolleyMotorSideState = false;
  }
}

void moveToMiddle()
{
  int32_t error = (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) / 2 - encoderTrolley.getPulse();
  int16_t pwm = 255 * error / ((ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) / 2);
  trolleyMotor.setPWM(pwm);
}

#endif // UTILITY_H