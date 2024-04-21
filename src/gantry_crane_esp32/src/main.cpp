#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.hpp"
#include "ADC.hpp"

TaskHandle_t controllerCommandTaskHandle;

TaskHandle_t spinMicroROSTaskHandle;

void controllerCommandTask(void *parameter);

void spinMicroROS(void *parameter);

void findOrigin();

void setup()
{

  // Serial.begin(115200);

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
      25000,
      NULL,
      1,
      &controllerCommandTaskHandle,
      1);

  // xTaskCreatePinnedToCore(
  //     spinMicroROS,
  //     "microROS spin task",
  //     30000,
  //     NULL,
  //     0,
  //     &spinMicroROSTaskHandle,
  //     0);
}

unsigned long lastTime = 0;
bool inside = false;
void loop()
{
  spinMicroROS(NULL);
  // controllerCommandTask(NULL);
}

void spinMicroROS(void *parameter)
{
  for (;;)
  {
    ledBuiltIn.pulse(500);

    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);

    RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&controllerCommandExecutor, RCL_MS_TO_NS(CONTROLLER_COMMAND_SUBSCRIBER_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&trolleyMotorVoltageExecutor, RCL_MS_TO_NS(TROLLEY_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&hoistMotorVoltageExecutor, RCL_MS_TO_NS(HOIST_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
  }
}

void controllerCommandTask(void *parameter)
{
  for (;;)
  {
    checkLimitSwitch();

    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);

    if (brakeTrolleyMotor)
    {
      trolleyMotorPWM = 0;
      trolleyMotor.setPWM(trolleyMotorPWM);
      trolleyMotor.brake();
      brakeTrolleyMotor = false;
    }

    if (brakeHoistMotor)
    {
      hoistMotorPWM = 0;
      hoistMotor.setPWM(hoistMotorPWM);
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
      trolleyMotorPWM = trolleyMotorPWM/2;
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

    trolleyMotor.setPWM(trolleyMotorPWM);
    hoistMotor.setPWM(hoistMotorPWM);

    // trolleyMotorVoltageMovingAverage.addValue(readChannel(ADS1115_COMP_0_GND));
    // hoistMotorVoltageMovingAverage.addValue(readChannel(ADS1115_COMP_1_GND));
    // trolleyMotorVoltage = readChannel(ADS1115_COMP_0_GND);
    // hoistMotorVoltage = readChannel(ADS1115_COMP_1_GND);
  }
}

void findOrigin()
{
  
  int counter = 0;
  while (counter < 2)
  {
    // ledBuiltIn.pulse(250);
    // Serial.println("Finding origin");
    if (limitSwitchEncoderSide.getState() != LOW)
    {
      trolleyMotorPWM = -TROLLEY_MOTOR_FIND_ORIGIN_PWM;
      trolleyMotor.setPWM(trolleyMotorPWM);
      delay(10);
    }
    else
    {
      trolleyMotorPWM = 0;
      trolleyMotor.setPWM(trolleyMotorPWM);
      counter++;
      delay(1000);
    }

    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);
    positionMessage.data = trolleyPosition;
    RCSOFTCHECK(rcl_publish(&positionPublisher, &positionMessage, NULL));
  }

  counter = 0;
  while (counter < 2)
  {
    // ledBuiltIn.pulse(250);
    for (int i = -0.5 * TROLLEY_MOTOR_PWM_MAX; i > -(TROLLEY_MOTOR_FIND_ORIGIN_PWM + 50); i--)
    {
      trolleyMotorPWM = i;
      trolleyMotor.setPWM(trolleyMotorPWM);
      // delay(2);
    }

    trolleyMotorPWM = 0;
    trolleyMotor.setPWM(trolleyMotorPWM);
    counter++;
    // delay(100);
    encoderTrolley.reset();
    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);
    positionMessage.data = trolleyPosition;
    RCSOFTCHECK(rcl_publish(&positionPublisher, &positionMessage, NULL));
  }
}
