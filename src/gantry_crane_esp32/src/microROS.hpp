#ifndef MICRO_ROS_HPP
#define MICRO_ROS_HPP

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "ADC.hpp"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This script is only available for Arduino framework with serial transport.
#endif

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      gantryMode = IDLE_MODE;    \
    }                            \
  }

// Error handle loop
void error_loop()
{
  ledBuiltIn.turnOn();

  vTaskDelete(spinMicroROSTaskHandle);
  vTaskDelete(controllerCommandTaskHandle);
}
void trolleyPositionPubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // trolleyPosition = static_cast<float>(encoderTrolley.getPulse());
    positionMessage.data = trolleyPosition;
    RCSOFTCHECK(rcl_publish(&positionPublisher, &positionMessage, NULL));
  }
}

u_int32_t controller_command_last_call_time = 0;
int8_t last_gantry_mode = IDLE_MODE;
void controllerCommandHandler();
void controllerCommandCallback(const void *msgin)
{
  const std_msgs__msg__UInt32 *controllerCommandMessage = (const std_msgs__msg__UInt32 *)msgin;
  unpackValues(controllerCommandMessage->data, gantryMode, trolleyMotorPWM, hoistMotorPWM);
  controllerCommandHandler();
  controller_command_last_call_time = millis();
  last_gantry_mode = gantryMode;
}

void controllerCommandHandler()
{
  if (trolleyMotorPWM == BRAKE_COMMAND)
  {
    trolleyMotorPWM = 0;
    brakeTrolleyMotor = true;
  }
  else
  {
    brakeTrolleyMotor = false;
  }

  if (hoistMotorPWM == BRAKE_COMMAND)
  {
    hoistMotorPWM = 0;
    brakeHoistMotor = true;
  }
  else
  {
    brakeHoistMotor = false;
  }

  if (limitSwitchEncoderSideState && trolleyMotorPWM < 0)
  {
    trolleyMotorPWM = 0;
  }
  // if (limitSwitchTrolleyMotorSideState && trolleyMotorPWM > 0)
  // {
  //   trolleyMotorPWM = 0;
  // }
}

void trolleyMotorVoltagePubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    float trolleyMotorVoltage_ = trolleyMotorVoltage * 3.173492867;
    if (trolleyMotorPWM < 0 || trolleySpeed < 0 || lastTrolleyMotorVoltage < 0.0)
    {
      trolleyMotorVoltage = -fabs(trolleyMotorVoltage);
    }
    if (fabs(trolleyMotorVoltage) < 0.005)
    {
      trolleyMotorVoltage = 0;
    }
    trolleyMotorVoltageMessage.data = roundToSixDecimalPlaces(trolleyMotorVoltage_);
    // trolleyMotorVoltageMessage.data = trolleyMotor.currentPWM;
    RCSOFTCHECK(rcl_publish(&trolleyMotorVoltagePublisher, &trolleyMotorVoltageMessage, NULL));
    lastTrolleyMotorVoltage = trolleyMotorVoltage;
    if ((trolleyMotorPWM >= 0) || (lastTrolleyMotorVoltage > -0.5))
    {
      lastTrolleyMotorVoltage = 0;
    }
  }
}

void hoistMotorVoltagePubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    float hoistMotorVoltage_ = hoistMotorVoltage * 3.13675602;
    if (hoistMotorPWM < 0 || lastHoistMotorVoltage < 0.0)
    {
      hoistMotorVoltage = -fabs(hoistMotorVoltage);
    }
    if (fabs(hoistMotorVoltage) < 0.005)
    {
      hoistMotorVoltage = 0;
    }
    lastHoistMotorVoltage = hoistMotorVoltage;
    if ((hoistMotorPWM >= 0) || (lastHoistMotorVoltage > -0.5))
    {
      lastHoistMotorVoltage = 0;
    }

    hoistMotorVoltageMessage.data = roundToSixDecimalPlaces(hoistMotorVoltage_);
    // hoistMotorVoltageMessage.data = hoistMotor.currentPWM;
    RCSOFTCHECK(rcl_publish(&hoistMotorVoltagePublisher, &hoistMotorVoltageMessage, NULL));
    
    
  }
}

void initTrolleyPositionPublisher()
{
  // trolley position publisher init
  RCCHECK(rclc_publisher_init_default(
      &positionPublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      TROLLEY_POSITION_TOPIC_NAME));

  // trolley position timer init
  RCCHECK(rclc_timer_init_default(
      &positionPubTimer,
      &support,
      RCL_MS_TO_NS(POSITION_PUBLISH_PERIOD_MS),
      trolleyPositionPubTimerCallback));

  // trolley position executor init
  RCCHECK(rclc_executor_init(&positionPubExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&positionPubExecutor, &positionPubTimer));
}

void initControllerCommandSubscriber()
{
  // motor PWM subscriber init
  RCCHECK(rclc_subscription_init_default(
      &controllerCommandSubscriber,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      CONTROLLER_COMMAND_TOPIC_NAME));

  // motor PWM executor init
  RCCHECK(rclc_executor_init(&controllerCommandExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&controllerCommandExecutor, &controllerCommandSubscriber, &controllerCommandMessage, &controllerCommandCallback, ON_NEW_DATA));
}

void initTrolleyMotorVoltagePublisher()
{
  // trolley motor voltage publisher init
  RCCHECK(rclc_publisher_init_default(
      &trolleyMotorVoltagePublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME));

  // trolley motor voltage timer init
  RCCHECK(rclc_timer_init_default(
      &trolleyMotorVoltagePubTimer,
      &support,
      RCL_MS_TO_NS(TROLLEY_MOTOR_VOLTAGE_PUBLISH_PERIOD_MS),
      trolleyMotorVoltagePubTimerCallback));

  // trolley motor voltage executor init
  RCCHECK(rclc_executor_init(&trolleyMotorVoltageExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&trolleyMotorVoltageExecutor, &trolleyMotorVoltagePubTimer));
}

void initHoistMotorVoltagePublisher()
{
  // hoist motor voltage publisher init
  RCCHECK(rclc_publisher_init_default(
      &hoistMotorVoltagePublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      HOIST_MOTOR_VOLTAGE_TOPIC_NAME));

  // hoist motor voltage timer init
  RCCHECK(rclc_timer_init_default(
      &hoistMotorVoltagePubTimer,
      &support,
      RCL_MS_TO_NS(HOIST_MOTOR_VOLTAGE_PUBLISH_PERIOD_MS),
      hoistMotorVoltagePubTimerCallback));

  // hoist motor voltage publisher
  RCCHECK(rclc_executor_init(&hoistMotorVoltageExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&hoistMotorVoltageExecutor, &hoistMotorVoltagePubTimer));
}

void microROSInit()
{
  // Configure serial transport
  Serial.begin(SERIAL_BAUDRATE);
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&microcontrollerGantryNode, NODE_NAME, "", &support));

  // init motor PWM subscriber
  initControllerCommandSubscriber();

  // init trolley position publisher
  initTrolleyPositionPublisher();

  // init trolley motor voltage publisher
  initTrolleyMotorVoltagePublisher();

  // init hoist motor voltage publisher
  initHoistMotorVoltagePublisher();
}

#endif // MICRO_ROS_HPP