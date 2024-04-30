#ifndef VARIABLE_H
#define VARIABLE_H

#include <LED.h>
#include <Wire.h>
#include <motor.hpp>
#include <encoder.hpp>
#include <debounce.hpp>
#include <ADS1115_WE.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>

#include "parameter.h"

Motor trolleyMotor(TROLLEY_MOTOR_FORWARD_PIN, TROLLEY_MOTOR_REVERSE_PIN, TROLLEY_MOTOR_PWM_PIN, TROLLEY_MOTOR_PWM_MIN, TROLLEY_MOTOR_PWM_MAX);
Motor hoistMotor(HOIST_MOTOR_FORWARD_PIN, HOIST_MOTOR_REVERSE_PIN, HOIST_MOTOR_PWM_PIN, HOIST_MOTOR_PWM_MIN, HOIST_MOTOR_PWM_MAX);

Encoder encoderTrolley(ENCODER_CHANNEL_A_PIN, ENCODER_CHANNEL_B_PIN, ENCODER_MIN_VALUE, ENCODER_MAX_VALUE, POSITION_MIN_VALUE, POSITION_MAX_VALUE);
void encoderTrolleyCallback(){
  encoderTrolley.update();
}

Debounce limitSwitchEncoderSide(LIMIT_SWITCH_ENCODER_SIDE_PIN);
Debounce limitSwitchTrolleyMotorSide(LIMIT_SWITCH_TROLLEY_MOTOR_SIDE_PIN);

LED ledBuiltIn(LED_BUILTIN);

ADS1115_WE analogToDigitalConverter = ADS1115_WE(ADS1115_I2C_ADDRESS);

int8_t gantryMode = 0;

int16_t trolleyMotorPWM, hoistMotorPWM;

float trolleyMotorVoltage = 0;
float lastTrolleyMotorVoltage = 0;

float hoistMotorVoltage = 0;
float lastHoistMotorVoltage = 0;

bool brakeTrolleyMotor = false;
bool brakeHoistMotor = false;

bool limitSwitchEncoderSideState = false;
bool limitSwitchTrolleyMotorSideState = false;

TaskHandle_t spinMicroROSTaskHandle;
TaskHandle_t controllerCommandTaskHandle;

SemaphoreHandle_t initmicroROSSemaphore;

float trolleyPosition = 0;
float lastTrolleyPosition = 0;

volatile float trolleySpeed = 0;

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

void microROSInit();

void spinMicroROS(void *parameter);

void controllerCommandTask(void *parameter);

#endif // VARIABLE_H