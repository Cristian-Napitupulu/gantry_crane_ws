#ifndef MICRO_ROS_H
#define MICRO_ROS_H

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This script is only available for Arduino framework with serial transport.
#endif

// Function prototypes
void error_loop();
void encoderPubTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
void motorPWMCallback(const void *msgin);
void microROSInit();

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
std_msgs__msg__Int32 motorPWMMessage;
rclc_executor_t motorPWMExecutor;

// Error handle loop
void error_loop()
{
  while (1)
  {
    ledBuiltIn.blink(100);
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

// Timer callback function for encoder publisher
// Encoder publisher uses a timer to avoid flooding the network
void encoderPubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
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
  const std_msgs__msg__Int32 *motorPWMMessage = (const std_msgs__msg__Int32 *)msgin;
  int16_t trolleyMotorPWM, hoistMotorPWM;
  splitInt32toInt16(motorPWMMessage->data, trolleyMotorPWM, hoistMotorPWM);
  if (limitSwitchEncoderSideState || limitSwitchTrolleyMotorSideState)
  {
    trolleyMotor.setPWM(0);
    hoistMotor.setPWM(0);
  }
  else
  {
    trolleyMotor.setPWM(trolleyMotorPWM);
    hoistMotor.setPWM(hoistMotorPWM);
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
  RCCHECK(rclc_publisher_init_default(
      &limitSwitchPublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      LIMIT_SWITCH_TOPIC_NAME));

  // create encoder publisher
  RCCHECK(rclc_publisher_init_default(
      &positionPublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      POSITION_TOPIC_NAME));

  // create timer
  RCCHECK(rclc_timer_init_default(
      &limitSwitchPubTimer,
      &support,
      RCL_MS_TO_NS(LIMIT_SWITCH_PUBLISH_PERIOD_MS),
      publishLimitSwitchState));

  RCCHECK(rclc_timer_init_default(
      &positionPubTimer,
      &support,
      RCL_MS_TO_NS(POSITION_PUBLISH_PERIOD_MS),
      encoderPubTimerCallback));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &motorPWMSubscriber,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      MOTOR_PWM_TOPIC_NAME));

  // create executor
  RCCHECK(rclc_executor_init(&limitSwitchExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&limitSwitchExecutor, &limitSwitchPubTimer));

  RCCHECK(rclc_executor_init(&positionPubExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&positionPubExecutor, &positionPubTimer));

  RCCHECK(rclc_executor_init(&motorPWMExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&motorPWMExecutor, &motorPWMSubscriber, &motorPWMMessage, &motorPWMCallback, ON_NEW_DATA));
}

#endif // MICRO_ROS_H