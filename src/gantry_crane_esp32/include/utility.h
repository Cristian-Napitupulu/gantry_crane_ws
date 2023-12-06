#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>
#include "variable.h"

void unpackValues(int32_t inputInt32, int8_t &mode, int16_t &pwm_trolley, int16_t &pwm_hoist)
{
  // Extract the values from the packed 32-bit integer
    mode = static_cast<int8_t>(inputInt32 & 0xFF);
    pwm_trolley = static_cast<int16_t>((inputInt32 >> 8) & 0xFFF);
    pwm_hoist = static_cast<int16_t>((inputInt32 >> 20) & 0xFFF);

    // Convert two's complement representation back to negative values
    if (pwm_trolley & 0x800) {
        pwm_trolley = pwm_trolley - 0xFFF - 1;
    }
    if (pwm_hoist & 0x800) {
        pwm_hoist = pwm_hoist - 0xFFF - 1;
    }
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