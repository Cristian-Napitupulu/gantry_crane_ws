#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.hpp"
#include "ADC.hpp"

TaskHandle_t controllerCommandTaskHandle;

void controllerCommandTask(void *parameter);

void findOrigin();

void spinMicroROS(void *parameter)
{
  for (;;)
  {
    RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&controllerCommandExecutor, RCL_MS_TO_NS(MOTOR_PWM_SUBSCRIBER_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&trolleyMotorVoltageExecutor, RCL_MS_TO_NS(TROLLEY_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&hoistMotorVoltageExecutor, RCL_MS_TO_NS(HOIST_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
    // RCSOFTCHECK(rclc_executor_spin_some(&limitSwitchExecutor, RCL_MS_TO_NS(LIMIT_SWITCH_PUBLISH_TIMEOUT_MS)));
  }
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

  // Initialize ADC
  analogToDigitalConverterInit();

  // Initialize microROS
  microROSInit();

  // Find origin
  findOrigin();

  ledBuiltIn.turnOff();

  gantryMode = IDLE_MODE;

  xTaskCreatePinnedToCore(
      controllerCommandTask,
      "Controller command task",
      10000,
      NULL,
      1,
      &controllerCommandTaskHandle,
      0);
}

unsigned long lastTime = 0;
bool inside = false;
void loop()
{
  spinMicroROS(NULL);
}

void controllerCommandTask(void *parameter)
{
  for (;;)
  {
    ledBuiltIn.pulse();

    checkLimitSwitch();

    if (brakeTrolleyMotor)
    {
      trolleyMotor.brake();
    }
    else
    {
      trolleyMotor.setPWM(trolleyMotorPWM);
    }

    if (brakeHoistMotor)
    {
      hoistMotor.brake();
    }
    else
    {
      hoistMotor.setPWM(hoistMotorPWM);
    }

    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);

    trolleyMotorVoltageMovingAverage.addValue(readChannel(ADS1115_COMP_0_GND));
    hoistMotorVoltageMovingAverage.addValue(readChannel(ADS1115_COMP_1_GND));

    if (gantryMode == IDLE_MODE)
    {
      // Do nothing
      trolleyMotorPWM = 0;
      hoistMotorPWM = 0;
    }

    else if (gantryMode == CONTROL_MODE)
    {
      // Control gantry
    }

    else if (gantryMode == LOCK_CONTAINER_MODE)
    {
      // Lock container
    }

    else if (gantryMode == UNLOCK_CONTAINER_MODE)
    {
      // Unlock container
    }

    else if (gantryMode == COLLECT_DATA_MODE)
    {
      // Collect data
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
      trolleyMotorPWM = static_cast<int16_t>(pidTrolley.calculate(ENCODER_MAX_VALUE / 2, encoderTrolley.getPulse()));
      hoistMotorPWM = 0;

      if (!inside && (fabs(encoderTrolley.getPulse() - ENCODER_MAX_VALUE / 2) < 0.001 * ENCODER_MAX_VALUE))
      {
        lastTime = millis();
        inside = true;
      }
      if (inside && (fabs(encoderTrolley.getPulse() - ENCODER_MAX_VALUE / 2) > 0.001 * ENCODER_MAX_VALUE))
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
  }
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
      trolleyMotor.setPWM(-TROLLEY_MOTOR_FIND_ORIGIN_PWM);
    }
    else
    {
      trolleyMotor.setPWM(0);
      counter++;
      delay(500);
    }
    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);
    positionMessage.data = trolleyPosition;
    RCSOFTCHECK(rcl_publish(&positionPublisher, &positionMessage, NULL));
  }
  delay(1000);
  counter = 0;
  while (counter < 2)
  {
    ledBuiltIn.blink(100);
    for (int i = -0.1 * TROLLEY_MOTOR_PWM_MAX; i > -TROLLEY_MOTOR_FIND_ORIGIN_PWM; i--)
    {
      trolleyMotor.setPWM(i);
      delay(1);
    }
    trolleyMotor.setPWM(0);
    counter++;
    delay(500);
    encoderTrolley.reset();
    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);
    positionMessage.data = trolleyPosition;
    RCSOFTCHECK(rcl_publish(&positionPublisher, &positionMessage, NULL));
  }
}
