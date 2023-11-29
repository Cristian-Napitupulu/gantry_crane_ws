#ifndef MICRO_ROS_H
#define MICRO_ROS_H

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int64.h>

#include "parameter.h"
#include "component.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This script is only available for Arduino framework with serial transport.
#endif

// Function prototypes
void error_loop();
void encoderPubTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
void trolleyPWMCallback(const void *msgin);
void hoistPWMCallback(const void *msgin);
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

// encoder publisher, timer, and executor
rcl_publisher_t encoderPublisher;
std_msgs__msg__Int64 encoderMessage;
rclc_executor_t encoderPubExecutor;
rcl_timer_t encoderPubTimer;

// motor PWM subscribers and executors
rcl_subscription_t trolleyPWMSubscriber;
std_msgs__msg__Int16 trolleyPWMMessage;
rclc_executor_t trolleyPWMExecutor;

rcl_subscription_t hoistPWMSubscriber;
std_msgs__msg__Int16 hoistPWMMessage;
rclc_executor_t hoistPWMExecutor;

// Error handle loop
void error_loop()
{
  while (1)
  {
  }
}

// Timer callback function for encoder publisher
// Encoder publisher uses a timer to avoid flooding the network
void encoderPubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    encoderMessage.data = encoderTrolley.getPulse();
    RCSOFTCHECK(rcl_publish(&encoderPublisher, &encoderMessage, NULL));
  }
}

// Subscriber callback function for trolley motor PWM
void trolleyPWMCallback(const void *msgin)
{
  const std_msgs__msg__Int16 *trolleyPWMMessage = (const std_msgs__msg__Int16 *)msgin;
  trolleyMotor.speed(trolleyPWMMessage->data);
}

// Subscriber callback function for hoist motor PWM
void hoistPWMCallback(const void *msgin)
{
  const std_msgs__msg__Int16 *hoistPWMMessage = (const std_msgs__msg__Int16 *)msgin;
  hoistMotor.speed(hoistPWMMessage->data);
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

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &encoderPublisher,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
      ENCODER_TOPIC_NAME));

  // create timer
  RCCHECK(rclc_timer_init_default(
      &encoderPubTimer,
      &support,
      RCL_MS_TO_NS(ENCODER_PUBLISH_PERIOD_MS),
      encoderPubTimerCallback));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &trolleyPWMSubscriber,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      TROLLEY_PWM_TOPIC_NAME));

  RCCHECK(rclc_subscription_init_default(
      &hoistPWMSubscriber,
      &microcontrollerGantryNode,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      HOIST_PWM_TOPIC_NAME));

  RCCHECK(rclc_executor_init(&encoderPubExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&encoderPubExecutor, &encoderPubTimer));

  RCCHECK(rclc_executor_init(&trolleyPWMExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&trolleyPWMExecutor, &trolleyPWMSubscriber, &trolleyPWMMessage, &trolleyPWMCallback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&hoistPWMExecutor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&hoistPWMExecutor, &hoistPWMSubscriber, &hoistPWMMessage, &hoistPWMCallback, ON_NEW_DATA));
}

#endif // MICRO_ROS_H