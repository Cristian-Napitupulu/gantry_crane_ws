#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>
#include "variable.h"

void unpackValues(uint32_t packedValue, int8_t &gantry_mode, int16_t &pwm_trolley, int16_t &pwm_hoist)
{
  // Extract the values from the packed 32-bit integer
  gantry_mode = static_cast<int8_t>(packedValue & 0xFF);
  pwm_trolley = static_cast<int16_t>((packedValue >> 8) & 0xFFF);
  pwm_hoist = static_cast<int16_t>((packedValue >> 20) & 0xFFF);

  // Convert two's complement representation back to negative values
  if (pwm_trolley & 0x800)
  {
    pwm_trolley = pwm_trolley - 0xFFF - 1;
  }
  if (pwm_hoist & 0x800)
  {
    pwm_hoist = pwm_hoist - 0xFFF - 1;
  }
}

float map_value(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float trolleySpeed = 0;
int32_t lastEncoderPulse = 0;
unsigned long lastEncoderTime = 0;

void checkLimitSwitch()
{
  // Check limit switch
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

  if (limitSwitchEncoderSideState && limitSwitchTrolleyMotorSideState)
  {
    trolleyMotorPWM = 0;
    hoistMotorPWM = 0;
  }

  // Protect system at high speed by reducing PWM by half
  if ((encoderTrolley.getPulse() > static_cast<int32_t>(0.8 * ENCODER_MAX_VALUE)) && (trolleyMotorPWM > 0.8 * TROLLEY_MOTOR_PWM_MAX))
  {
    trolleyMotorPWM = trolleyMotorPWM / 2;
  }

  if ((encoderTrolley.getPulse() < 0.2 * ENCODER_MAX_VALUE) && (trolleyMotorPWM < -0.8 * TROLLEY_MOTOR_PWM_MAX))
  {
    trolleyMotorPWM = trolleyMotorPWM / 2;
  }
}

#endif // UTILITY_H