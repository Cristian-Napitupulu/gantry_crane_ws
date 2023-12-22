#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.hpp"
#include "ADC.hpp"

TaskHandle_t controllerCommandTaskHandle;

TaskHandle_t spinMicroROSTaskHandle;

void controllerCommandTask(void *parameter);

void findOrigin();

void spinMicroROS(void *parameter)
{
  for (;;)
  {
    ledBuiltIn.pulse(500);

    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);

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

  // unsigned long start_time = millis();
  // while (true)
  // {
  //   ledBuiltIn.pulse(500);
  //   if (millis() - start_time > 5000)
  //   {
  //     break;
  //   }
  // }

  // Find origin
  findOrigin();

  ledBuiltIn.turnOff();

  gantryMode = IDLE_MODE;

  xTaskCreatePinnedToCore(
      controllerCommandTask,
      "Controller command task",
      20000,
      NULL,
      1,
      &controllerCommandTaskHandle,
      0);

  // xTaskCreatePinnedToCore(
  //     spinMicroROS,
  //     "microROS spin task",
  //     10000,
  //     NULL,
  //     1,
  //     &spinMicroROSTaskHandle,
  //     1);
  
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
    checkLimitSwitch();

    trolleyMotorVoltageMovingAverage.addValue(readChannel(ADS1115_COMP_0_GND));
    hoistMotorVoltageMovingAverage.addValue(readChannel(ADS1115_COMP_1_GND));

    trolleyMotor.setPWM(trolleyMotorPWM);
    hoistMotor.setPWM(hoistMotorPWM);

    switch (gantryMode)
    {
    case IDLE_MODE:
      // Do nothing
      trolleyMotorPWM = 0;
      hoistMotorPWM = 0;
      lastTrolleyMotorVoltage = 0;
      lastHoistMotorVoltage = 0;
      break;

    case CONTROL_MODE:
      // Control gantry
      // Add code specific to CONTROL_MODE
      break;

    case LOCK_CONTAINER_MODE:
      // Lock container
      // Add code specific to LOCK_CONTAINER_MODE
      break;

    case UNLOCK_CONTAINER_MODE:
      // Unlock container
      // Add code specific to UNLOCK_CONTAINER_MODE
      break;

    case MOVE_TO_ORIGIN_MODE:
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
      break;

    case MOVE_TO_MIDDLE_MODE:
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
      break;

    case MOVE_TO_END_MODE:
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
      break;

    default:
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
      delay(1000);
    }
    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);
    positionMessage.data = trolleyPosition;
    RCSOFTCHECK(rcl_publish(&positionPublisher, &positionMessage, NULL));
  }
  delay(500);
  counter = 0;
  while (counter < 2)
  {
    ledBuiltIn.blink(100);
    for (int i = -0.5 * TROLLEY_MOTOR_PWM_MAX; i > -(TROLLEY_MOTOR_FIND_ORIGIN_PWM + 50); i--)
    {
      trolleyMotor.setPWM(i);
      delay(2);
    }
    trolleyMotor.setPWM(0);
    counter++;
    delay(100);
    encoderTrolley.reset();
    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);
    positionMessage.data = trolleyPosition;
    RCSOFTCHECK(rcl_publish(&positionPublisher, &positionMessage, NULL));
  }
}
