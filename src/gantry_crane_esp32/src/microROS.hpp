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

int32_t counter = 0;

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
      counter++;                 \
    }                            \
    if (counter > 5)             \
    {                            \
      gantryMode = IDLE_MODE;    \
      counter = 0;               \
    }                            \
}

// Global variables
// micro-ROS node
rcl_node_t microcontrollerGantryNode;
rcl_allocator_t allocator;
rclc_support_t support;

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

TaskHandle_t microROSInitTaskHandle;
SemaphoreHandle_t microROSInitFinishSemaphore;

TaskHandle_t spinMicroROSTaskHandle;

// Function prototypes
void microROSInit();
void spinMicroROS(void *parameter);

// Error handle loop
void error_loop()
{
  ledBuiltIn.turnOn();
  microROSInit();
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

uint64_t controller_command_last_call_time = 0;
bool brakeTrolleyMotor = false;
bool brakeHoistMotor = false;
void controllerCommandHandler();
void controllerCommandCallback(const void *msgin)
{
  const std_msgs__msg__UInt32 *controllerCommandMessage = (const std_msgs__msg__UInt32 *)msgin;
  unpackValues(controllerCommandMessage->data, gantryMode, trolleyMotorPWM, hoistMotorPWM);
  controllerCommandHandler();
  controller_command_last_call_time = millis() / 1000;
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

volatile float lastTrolleyMotorVoltage = 0;
volatile float trolleyMotorVoltage = 0;
void trolleyMotorVoltagePubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    if (trolleyMotorPWM < 0 || encoderTrolley.getPulsePerSecond() < 0 || lastTrolleyMotorVoltage < 0.0)
    {
      trolleyMotorVoltage = -trolleyMotorVoltage;
    }
    if (fabs(trolleyMotorVoltage) < 0.0001)
    {
      trolleyMotorVoltage = 0;
    }

    volatile float trolleyMotorVoltage_ = trolleyMotorVoltage * 3.1694367498;
    trolleyMotorVoltageMessage.data = trolleyMotorPWM;
    // trolleyMotorVoltageMessage.data = trolleyMotorPWM;
    RCSOFTCHECK(rcl_publish(&trolleyMotorVoltagePublisher, &trolleyMotorVoltageMessage, NULL));

    lastTrolleyMotorVoltage = trolleyMotorVoltage;
    if (trolleyMotorPWM >= 0 || lastTrolleyMotorVoltage > -0.5)
    {
      lastTrolleyMotorVoltage = 0;
    }
  }
}

volatile float lastHoistMotorVoltage = 0;
volatile float hoistMotorVoltage = 0;
void hoistMotorVoltagePubTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    if (hoistMotorPWM < 0 || lastHoistMotorVoltage < 0.0)
    {
      hoistMotorVoltage = -hoistMotorVoltage;
    }
    if (fabs(hoistMotorVoltage) < 0.0001)
    {
      hoistMotorVoltage = 0;
    }
    volatile float hoistMotorVoltage_ = hoistMotorVoltage * 3.179049377;
    hoistMotorVoltageMessage.data = hoistMotorPWM;
    // hoistMotorVoltageMessage.data = hoistMotorPWM;
    RCSOFTCHECK(rcl_publish(&hoistMotorVoltagePublisher, &hoistMotorVoltageMessage, NULL));
    lastHoistMotorVoltage = hoistMotorVoltage;
    if (hoistMotorPWM >= 0 || lastHoistMotorVoltage > -0.5)
    {
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
  RCCHECK(rclc_executor_add_subscription(&controllerCommandExecutor, &controllerCommandSubscriber, &controllerCommandMessage, &controllerCommandCallback, ALWAYS));
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

void spinMicroROS(void *parameter)
{
  for (;;)
  {
    trolleyPosition = map_value(encoderTrolley.getPulse(), ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);

    RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&controllerCommandExecutor, RCL_MS_TO_NS(CONTROLLER_COMMAND_SUBSCRIBER_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&trolleyMotorVoltageExecutor, RCL_MS_TO_NS(TROLLEY_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
    RCSOFTCHECK(rclc_executor_spin_some(&hoistMotorVoltageExecutor, RCL_MS_TO_NS(HOIST_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
  }
}

#endif // MICRO_ROS_HPP