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

// motor PWM subscribers and executors
rcl_subscription_t motorPWMSubscriber;
std_msgs__msg__UInt32 motorPWMMessage;
rclc_executor_t motorPWMExecutor;

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

// Error handle loop
void error_loop()
{
  while (1)
  {
    ledBuiltIn.blink(1000);
  }
}

void publishLimitSwitchState(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
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

void trolleyPositionPubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    float position = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);
    positionMessage.data = position;
    RCSOFTCHECK(rcl_publish(&positionPublisher, &positionMessage, NULL));
  }
}

// Subscriber callback function for trolley motor PWM
void motorPWMCallback(const void *msgin)
{
  const std_msgs__msg__UInt32 *motorPWMMessage = (const std_msgs__msg__UInt32 *)msgin;
  unpackValues(motorPWMMessage->data, gantryMode, trolleyMotorPWM, hoistMotorPWM);
  if (limitSwitchEncoderSideState && trolleyMotorPWM < 0)
  {
    trolleyMotorPWM = 0;
  }
  if (limitSwitchTrolleyMotorSideState && trolleyMotorPWM > 0)
  {
    trolleyMotorPWM = 0;
  }
}

void trolleyMotorVoltagePubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    trolleyMotorVoltageMessage.data = readChannel(ADS1115_COMP_0_GND);
    if (trolleyMotorPWM < 0)
    {
      trolleyMotorVoltageMessage.data *= -1;
    }
    RCSOFTCHECK(rcl_publish(&trolleyMotorVoltagePublisher, &trolleyMotorVoltageMessage, NULL));
  }
}

void hoistMotorVoltagePubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    hoistMotorVoltageMessage.data = readChannel(ADS1115_COMP_1_GND);
    if (hoistMotorPWM < 0)
    {
      hoistMotorVoltageMessage.data *= -1;
    }
    RCSOFTCHECK(rcl_publish(&hoistMotorVoltagePublisher, &hoistMotorVoltageMessage, NULL));
  }
}

void microROSInit()
{
  // Configure serial transport
  Serial.begin(SERIAL_BAUDRATE);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&microcontrollerGantryNode, NODE_NAME, "", &support));

  // create limit switches publisher
  // RCCHECK(rclc_publisher_init_default(
  //     &limitSwitchPublisher,
  //     &microcontrollerGantryNode,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
  //     LIMIT_SWITCH_TOPIC_NAME));

  // create trolley position publisher
  RCCHECK(rclc_publisher_init_default(
      &positionPublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      TROLLEY_POSITION_TOPIC_NAME));

  // create trolley motor voltage publisher
  RCCHECK(rclc_publisher_init_default(
      &trolleyMotorVoltagePublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME));

  // create hoist motor voltage publisher
  RCCHECK(rclc_publisher_init_default(
      &hoistMotorVoltagePublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      HOIST_MOTOR_VOLTAGE_TOPIC_NAME));

  // create timer
  // limit switch publisher
  // RCCHECK(rclc_timer_init_default(
  //     &limitSwitchPubTimer,
  //     &support,
  //     RCL_MS_TO_NS(LIMIT_SWITCH_PUBLISH_PERIOD_MS),
  //     publishLimitSwitchState));

  // trolley position publisher
  RCCHECK(rclc_timer_init_default(
      &positionPubTimer,
      &support,
      RCL_MS_TO_NS(POSITION_PUBLISH_PERIOD_MS),
      trolleyPositionPubTimerCallback));

  // trolley motor voltage publisher
  RCCHECK(rclc_timer_init_default(
      &trolleyMotorVoltagePubTimer,
      &support,
      RCL_MS_TO_NS(TROLLEY_MOTOR_VOLTAGE_PUBLISH_PERIOD_MS),
      trolleyMotorVoltagePubTimerCallback));

  // hoist motor voltage publisher
  RCCHECK(rclc_timer_init_default(
      &hoistMotorVoltagePubTimer,
      &support,
      RCL_MS_TO_NS(HOIST_MOTOR_VOLTAGE_PUBLISH_PERIOD_MS),
      hoistMotorVoltagePubTimerCallback));

  // create subscriber
  // motor PWM subscriber
  RCCHECK(rclc_subscription_init_default(
      &motorPWMSubscriber,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      MOTOR_PWM_TOPIC_NAME));

  // create executor
  // limit switch publisher
  // RCCHECK(rclc_executor_init(&limitSwitchExecutor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&limitSwitchExecutor, &limitSwitchPubTimer));

  // trolley position publisher
  RCCHECK(rclc_executor_init(&positionPubExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&positionPubExecutor, &positionPubTimer));

  // trolley motor voltage publisher
  RCCHECK(rclc_executor_init(&trolleyMotorVoltageExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&trolleyMotorVoltageExecutor, &trolleyMotorVoltagePubTimer));

  // hoist motor voltage publisher
  RCCHECK(rclc_executor_init(&hoistMotorVoltageExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&hoistMotorVoltageExecutor, &hoistMotorVoltagePubTimer));

  // motor PWM subscriber
  RCCHECK(rclc_executor_init(&motorPWMExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&motorPWMExecutor, &motorPWMSubscriber, &motorPWMMessage, &motorPWMCallback, ON_NEW_DATA));
}

#endif // MICRO_ROS_HPP