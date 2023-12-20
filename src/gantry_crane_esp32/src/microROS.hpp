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
#include "lookUpTable.hpp"

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
    }                            \
  }

// Global variables
// micro-ROS node
rcl_node_t microcontrollerGantryNode;
rcl_allocator_t allocator;
rclc_support_t support;

// limit switches
rcl_publisher_t limitSwitchPublisher;
std_msgs__msg__Int8 limitSwitchMessage;
rclc_executor_t limitSwitchExecutor;
rcl_timer_t limitSwitchPubTimer;

// encoder publisher, timer, and executor
rcl_publisher_t positionPublisher;
std_msgs__msg__Float32 positionMessage;
rclc_executor_t positionPubExecutor;
rcl_timer_t positionPubTimer;

// controller command subscribers and executors
rcl_subscription_t controllerCommandSubscriber;
std_msgs__msg__UInt32 controllerCommandMessage;
rclc_executor_t controllerCommandExecutor;

// Trolley Motor Voltage Publisher
rcl_publisher_t trolleyMotorVoltagePublisher;
std_msgs__msg__Float32 trolleyMotorVoltageMessage;
rclc_executor_t trolleyMotorVoltageExecutor;
rcl_timer_t trolleyMotorVoltagePubTimer;

// Hoist Motor Voltage Publisher
rcl_publisher_t hoistMotorVoltagePublisher;
std_msgs__msg__Float32 hoistMotorVoltageMessage;
rclc_executor_t hoistMotorVoltageExecutor;
rcl_timer_t hoistMotorVoltagePubTimer;

void microROSInit();

// Error handle loop
void error_loop()
{
  ledBuiltIn.turnOn();
  microROSInit();
}

void publishLimitSwitchState(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    if (limitSwitchEncoderSideState && limitSwitchTrolleyMotorSideState)
    {
      limitSwitchMessage.data = LIMIT_SWITCH_BOTH_TRIGGERED;
    }
    else if (limitSwitchEncoderSideState && !limitSwitchTrolleyMotorSideState)
    {
      limitSwitchMessage.data = LIMIT_SWITCH_ENCODER_SIDE_TRIGGERED;
    }
    else if (!limitSwitchEncoderSideState && limitSwitchTrolleyMotorSideState)
    {
      limitSwitchMessage.data = LIMIT_SWITCH_TROLLEY_MOTOR_SIDE_TRIGGERED;
    }
    else
    {
      limitSwitchMessage.data = LIMIT_SWITCH_NONE_TRIGGERED;
    }

    RCSOFTCHECK(rcl_publish(&limitSwitchPublisher, &limitSwitchMessage, NULL));
  }
}

float trolleyPosition = 0;
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

bool brakeTrolleyMotor = false;
bool brakeHoistMotor = false;
void controllerCommandHandler();
void controllerCommandCallback(const void *msgin)
{
  const std_msgs__msg__UInt32 *controllerCommandMessage = (const std_msgs__msg__UInt32 *)msgin;
  unpackValues(controllerCommandMessage->data, gantryMode, trolleyMotorPWM, hoistMotorPWM);
  controllerCommandHandler();
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
  if (limitSwitchTrolleyMotorSideState && trolleyMotorPWM > 0)
  {
    trolleyMotorPWM = 0;
  }
}

float lastTrolleyMotorVoltage = 0;
void trolleyMotorVoltagePubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    float trolleyMotorVoltage = trolleyMotorVoltageMovingAverage.getMovingAverage() * 3.1694367498;
    if (trolleyMotorPWM < 0 || lastTrolleyMotorVoltage < 0.0)
    {
      trolleyMotorVoltage = -trolleyMotorVoltage;
    }
    trolleyMotorVoltageMessage.data = trolleyMotorVoltage;
    RCSOFTCHECK(rcl_publish(&trolleyMotorVoltagePublisher, &trolleyMotorVoltageMessage, NULL));
    lastTrolleyMotorVoltage = trolleyMotorVoltage;
    if (trolleyMotorPWM == 0 && lastTrolleyMotorVoltage > -0.1){
      lastTrolleyMotorVoltage = 0;
    }
  }
}

float lastHoistMotorVoltage = 0;
void hoistMotorVoltagePubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    float hoistMotorVoltage = hoistMotorVoltageMovingAverage.getMovingAverage() * 3.179049377;
    if (hoistMotorPWM < 0 || lastHoistMotorVoltage < 0.0)
    {
      hoistMotorVoltage = -hoistMotorVoltage;
    }
    hoistMotorVoltageMessage.data = hoistMotorVoltage;
    RCSOFTCHECK(rcl_publish(&hoistMotorVoltagePublisher, &hoistMotorVoltageMessage, NULL));
    lastHoistMotorVoltage = hoistMotorVoltage;
    if (hoistMotorPWM == 0 && lastHoistMotorVoltage > -0.1){
      lastHoistMotorVoltage = 0;
    }
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

void initMotorPWMSubscriber()
{
  // motor PWM subscriber init
  RCCHECK(rclc_subscription_init_default(
      &controllerCommandSubscriber,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      MOTOR_PWM_TOPIC_NAME));

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

void initLimitSwitchPublisher()
{
  // limit switches publisher init
  RCCHECK(rclc_publisher_init_default(
      &limitSwitchPublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      LIMIT_SWITCH_TOPIC_NAME));

  // limit switch timer init
  RCCHECK(rclc_timer_init_default(
      &limitSwitchPubTimer,
      &support,
      RCL_MS_TO_NS(LIMIT_SWITCH_PUBLISH_PERIOD_MS),
      publishLimitSwitchState));

  // limit switch executor init
  RCCHECK(rclc_executor_init(&limitSwitchExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&limitSwitchExecutor, &limitSwitchPubTimer));
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
  initMotorPWMSubscriber();

  // init trolley position publisher
  initTrolleyPositionPublisher();

  // init trolley motor voltage publisher
  initTrolleyMotorVoltagePublisher();

  // init hoist motor voltage publisher
  initHoistMotorVoltagePublisher();

  // init limit switch publisher
  // initLimitSwitchPublisher();
}

#endif // MICRO_ROS_HPP