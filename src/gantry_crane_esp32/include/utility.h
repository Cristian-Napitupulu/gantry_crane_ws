#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>
#include "variable.h"
#include "parameter.h"

void unpackValues(uint32_t packedValue, int8_t &gantry_mode, int16_t &pwm_trolley, int16_t &pwm_hoist)
{
  // Extract the values from the packed 32-bit integer
  gantry_mode = static_cast<int8_t>(packedValue & 0xFF);

  if (gantry_mode < 0)
  {
    gantry_mode = IDLE_MODE;
  }

  else if (gantry_mode == CONTROL_MODE)
  {
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
}

float map_value(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int get_sign(int x)
{
  if (x > 0)
  {
    return 1;
  }
  else if (x < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

void checkLimitSwitch()
{
  // Check limit switch
  if (limitSwitchEncoderSide.getState() == LOW)
  {
    encoderTrolley.reset();
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
    // encoderTrolley.setPulse(ENCODER_MAX_VALUE);
    // if (trolleyMotorPWM > 0)
    // {
    //   trolleyMotorPWM = 0;
    // }
  }
  else
  {
    limitSwitchTrolleyMotorSideState = false;
  }

  // if (limitSwitchEncoderSideState && limitSwitchTrolleyMotorSideState)
  // {
  //   trolleyMotorPWM = 0;
  //   hoistMotorPWM = 0;
  // }
}

uint64_t safetyCheckTimer = 0;
void safetyCheck()
{
  if (millis() - safetyCheckTimer > 25)
  {
    // Protect system at high speed by reducing PWM
    if ((trolleySpeed > TROLLEY_MAX_SPEED) && (encoderTrolley.getPulse() > static_cast<int32_t>(0.875 * ENCODER_MAX_VALUE)))
    {
      trolleyMotorPWM = trolleyMotorPWM * 0.995;
    }

    if ((trolleySpeed < -TROLLEY_MAX_SPEED) && (encoderTrolley.getPulse() < 0.125 * ENCODER_MAX_VALUE))
    {
      trolleyMotorPWM = trolleyMotorPWM * 0.995;
    }

    safetyCheckTimer = millis();
  }
}

float roundToThreeDecimalPlaces(float num)
{
  return round(num * 1000.0) / 1000.0;
}

#endif // UTILITY_H