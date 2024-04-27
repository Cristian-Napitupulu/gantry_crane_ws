#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.hpp"
#include "ADC.hpp"

TaskHandle_t controllerCommandTaskHandle;
TaskHandle_t findOriginTaskHandle;

void controllerCommandTask(void *parameter);

bool originFound = false;
void findOrigin(void *parameter);

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

  // Initialize ADC
  // analogToDigitalConverterInit();

  // Initialize microROS
  microROSInit();

  xTaskCreatePinnedToCore(
      spinMicroROS,
      "Spin Micro ROS Task",
      20000,
      NULL,
      1,
      &spinMicroROSTaskHandle,
      0);

  ledBuiltIn.turnOff();

  findOrigin(NULL);

  gantryMode = IDLE_MODE;

  xTaskCreatePinnedToCore(
      controllerCommandTask,
      "Controller Command Task",
      20000,
      NULL,
      1,
      &controllerCommandTaskHandle,
      1);
}

void loop()
{
  spinMicroROS(NULL);
  // controllerCommandTask(NULL);
}

void controllerCommandTask(void *parameter)
{
  for (;;)
  {
    ledBuiltIn.pulse(500);

    checkLimitSwitch();

    if (brakeTrolleyMotor)
    {
      trolleyMotorPWM = 0;
      trolleyMotor.setPWM(&trolleyMotorPWM);
      trolleyMotor.brake();
      brakeTrolleyMotor = false;
    }

    if (brakeHoistMotor)
    {
      hoistMotorPWM = 0;
      hoistMotor.setPWM(&hoistMotorPWM);
      hoistMotor.brake();
      brakeHoistMotor = false;
    }

    if (gantryMode == IDLE_MODE)
    {
      // Do nothing
      trolleyMotorPWM = 0;
      hoistMotorPWM = 0;
      lastTrolleyMotorVoltage = 0;
      lastHoistMotorVoltage = 0;

      trolleyMotor.brake();
      hoistMotor.brake();
    }
    else if (gantryMode == CONTROL_MODE)
    {
      // Control gantry
      u_int32_t current_time = millis() / 1000;
      if (current_time - controller_command_last_call_time >= CONTROLLER_COMMAND_TIMEOUT_MS)
      {
        gantryMode = IDLE_MODE;
      }
    }
    else if (gantryMode == LOCK_CONTAINER_MODE)
    {
      // Lock container
      // Add code specific to LOCK_CONTAINER_MODE
    }
    else if (gantryMode == UNLOCK_CONTAINER_MODE)
    {
      // Unlock container
      // Add code specific to UNLOCK_CONTAINER_MODE
    }
    else if (gantryMode == MOVE_TO_ORIGIN_MODE)
    {
      if (limitSwitchEncoderSide.getState() != LOW)
      {
        trolleyMotorPWM = -TROLLEY_MOTOR_FIND_ORIGIN_PWM;
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
      trolleyMotorPWM = TROLLEY_MOTOR_FIND_ORIGIN_PWM * get_sign(ENCODER_MAX_VALUE / 2 - encoderTrolley.getPulse());
      trolleyMotorPWM = trolleyMotorPWM / 2;
      hoistMotorPWM = 0;

      if (fabs(encoderTrolley.getPulse() - ENCODER_MAX_VALUE / 2) < 0.01 * ENCODER_MAX_VALUE)
      {
        trolleyMotorPWM = 0;
        gantryMode = IDLE_MODE;
      }
    }

    else if (gantryMode == MOVE_TO_END_MODE)
    {
      if (limitSwitchTrolleyMotorSide.getState() != LOW)
      {
        trolleyMotorPWM = TROLLEY_MOTOR_FIND_ORIGIN_PWM;
        hoistMotorPWM = 0;
      }
      else
      {
        trolleyMotorPWM = 0;
        gantryMode = IDLE_MODE;
      }
    }
    else
    {
      gantryMode = IDLE_MODE;
    }

    trolleyMotor.setPWM(&trolleyMotorPWM);
    hoistMotor.setPWM(&hoistMotorPWM);

    // trolleyMotorVoltageMovingAverage.addValue(readChannel(ADS1115_COMP_0_GND));
    // hoistMotorVoltageMovingAverage.addValue(readChannel(ADS1115_COMP_1_GND));
    // trolleyMotorVoltage = readChannel(ADS1115_COMP_0_GND);
    // hoistMotorVoltage = readChannel(ADS1115_COMP_1_GND);
  }
}

void findOrigin(void *parameter)
{
  for (int i = 0; i < 2; i++)
  {
    while (limitSwitchEncoderSide.getState() != LOW)
    {
      ledBuiltIn.pulse(125);
      trolleyMotorPWM = -(TROLLEY_MOTOR_FIND_ORIGIN_PWM + 50 * i);
      trolleyMotor.setPWM(&trolleyMotorPWM);
    }

    trolleyMotorPWM = 0;
    trolleyMotor.setPWM(&trolleyMotorPWM);
    delay(1500);
  }

  delay(1000);

  for (int i = 0; i < 2; i++)
  {
    for (int i = -0.5 * TROLLEY_MOTOR_FIND_ORIGIN_PWM; i > -(TROLLEY_MOTOR_FIND_ORIGIN_PWM + 100); i--)
    {
      trolleyMotorPWM = i;
      trolleyMotor.setPWM(&trolleyMotorPWM);
      // delay(2);
    }

    trolleyMotorPWM = 0;
    trolleyMotor.setPWM(&trolleyMotorPWM);

    delay(250);
    encoderTrolley.reset();
  }
}
