#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.hpp"
#include "ADC.hpp"

// float readChannel(ADS1115_MUX channel);

void findOrigin();

void collectData()
{
  trolleyMotor.setPWM(100);
  RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
  RCSOFTCHECK(rclc_executor_spin_some(&trolleyMotorVoltageExecutor, RCL_MS_TO_NS(TROLLEY_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
  RCSOFTCHECK(rclc_executor_spin_some(&hoistMotorVoltageExecutor, RCL_MS_TO_NS(HOIST_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
}

void setup()
{
  // Initialize notification LED
  ledBuiltIn.begin();
  ledBuiltIn.turnOn();

  // Initialize motor
  trolleyMotor.begin();
  hoistMotor.begin();

  // Initialize encoder
  encoderTrolley.begin();
  attachInterrupt(encoderTrolley.getChannelA(), encoderTrolleyCallback, CHANGE);

  // Initialize limit switches
  limitSwitchEncoderSide.begin();
  limitSwitchTrolleyMotorSide.begin();

  ledBuiltIn.turnOff();

  // Initialize microROS
  microROSInit();

  // Initialize ADC
  analogToDigitalConverterInit();

  // Find origin
  findOrigin();

  gantryMode = IDLE_MODE;
}
unsigned long lastTime = 0;
bool inside = false;
void loop()
{
  if (gantryMode == IDLE_MODE)
  {
    // Do nothing
    trolleyMotorPWM = 0;
    hoistMotorPWM = 0;
  }

  else if (gantryMode == COLLECT_DATA_MODE)
  {
    RCSOFTCHECK(rclc_executor_spin_some(&trolleyMotorVoltageExecutor, RCL_MS_TO_NS(TROLLEY_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&hoistMotorVoltageExecutor, RCL_MS_TO_NS(HOIST_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
  }

  else if (gantryMode == MOVE_TO_ORIGIN_MODE)
  {
    if (limitSwitchEncoderSide.getState() != LOW)
    {
      trolleyMotorPWM = -80;
      hoistMotorPWM = 0;
    }
    else
    {
      trolleyMotorPWM = 0;
      gantryMode = IDLE_MODE;
    }
  }

  else if (gantryMode == MOVE_TO_MIDDLE_MODE)
  {
    trolleyMotorPWM = static_cast<int16_t>(pidTrolley.calculate(ENCODER_MAX_VALUE / 2, encoderTrolley.getPulse()));
    hoistMotorPWM = 0;

    if (!inside && (fabs(encoderTrolley.getPulse() - ENCODER_MAX_VALUE / 2) < 0.1 * ENCODER_MAX_VALUE))
    {
      lastTime = millis();
      inside = true;
    }
    else if (inside && (fabs(encoderTrolley.getPulse() - ENCODER_MAX_VALUE / 2) > 0.1 * ENCODER_MAX_VALUE))
    {
      inside = false;
    }

    if (inside && (millis() - lastTime > 5000))
    {
      gantryMode = IDLE_MODE;
    }
  }

  else if (gantryMode == MOVE_TO_END_MODE)
  {
    if (limitSwitchTrolleyMotorSide.getState() != LOW)
    {
      trolleyMotorPWM = 80;
      hoistMotorPWM = 0;
    }
    else
    {
      trolleyMotorPWM = 0;
      gantryMode = IDLE_MODE;
    }
  }

  else if (gantryMode == LOCK_CONTAINER_MODE)
  {
    // Lock container
  }

  else if (gantryMode == UNLOCK_CONTAINER_MODE)
  {
    // Unlock container
  }

  else
  {
    gantryMode = IDLE_MODE;
  }

  ledBuiltIn.pulse();

  checkLimitSwitch();

  trolleyMotor.setPWM(trolleyMotorPWM);
  hoistMotor.setPWM(hoistMotorPWM);

  RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
  RCSOFTCHECK(rclc_executor_spin_some(&motorPWMExecutor, RCL_MS_TO_NS(MOTOR_PWM_SUBSCRIBER_TIMEOUT_MS)));
}

void findOrigin()
{
  int counter = 0;
  while (counter < 2)
  {
    Serial.println("Finding origin");
    ledBuiltIn.blink(100);
    if (limitSwitchEncoderSide.getState() != LOW)
    {
      trolleyMotor.setPWM(-80);
    }
    else
    {
      trolleyMotor.setPWM(0);
      counter++;
      delay(500);
    }
    RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
  }
  delay(1000);
  counter = 0;
  while (counter < 2)
  {
    ledBuiltIn.blink(100);
    for (int i = -60; i > -100; i--)
    {
      trolleyMotor.setPWM(i);
      delay(10);
    }
    trolleyMotor.setPWM(0);
    counter++;
    delay(500);
    encoderTrolley.reset();
    RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
  }
}
