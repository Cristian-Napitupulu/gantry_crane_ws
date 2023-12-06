#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.hpp"
#include "ADC.hpp"

float readChannel(ADS1115_MUX channel);

void findOrigin();

int mode = 0;
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
}

void loop()
{
  ledBuiltIn.pulse();

  checkLimitSwitch();

  trolleyMotor.setPWM(trolleyMotorPWM);
  hoistMotor.setPWM(hoistMotorPWM);

  RCSOFTCHECK(rclc_executor_spin_some(&limitSwitchExecutor, RCL_MS_TO_NS(LIMIT_SWITCH_PUBLISH_TIMEOUT_MS)));
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
      trolleyMotor.setPWM(-90);
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
    for (int i = -60; i > -120; i--)
    {
      trolleyMotor.setPWM(i);
      delay(10);
    }
    encoderTrolley.reset();
    trolleyMotor.setPWM(0);
    counter++;
    delay(500);
    RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
  }
}
