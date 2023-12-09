#ifndef VARIABLE_H
#define VARIABLE_H

#include <LED.h>
#include <Wire.h>
#include <motor.hpp>
#include <encoder.hpp>
#include <debounce.hpp>
#include <ADS1115_WE.h>
#include <pid.hpp>

#include "parameter.h"

Motor trolleyMotor(TROLLEY_MOTOR_FORWARD_PIN, TROLLEY_MOTOR_REVERSE_PIN, TROLLEY_MOTOR_PWM_PIN, TROLLEY_MOTOR_PWM_MIN, TROLLEY_MOTOR_PWM_MAX);
Motor hoistMotor(HOIST_MOTOR_FORWARD_PIN, HOIST_MOTOR_REVERSE_PIN, HOIST_MOTOR_PWM_PIN, HOIST_MOTOR_PWM_MIN, HOIST_MOTOR_PWM_MAX);
int16_t trolleyMotorPWM, hoistMotorPWM;

int8_t gantryMode = 0;

Encoder encoderTrolley(ENCODER_CHANNEL_A_PIN, ENCODER_CHANNEL_B_PIN);
void encoderTrolleyCallback(){
  encoderTrolley.update();
}

Debounce limitSwitchEncoderSide(LIMIT_SWITCH_ENCODER_SIDE_PIN);
Debounce limitSwitchTrolleyMotorSide(LIMIT_SWITCH_TROLLEY_MOTOR_SIDE_PIN);

bool limitSwitchEncoderSideState = false;
bool limitSwitchTrolleyMotorSideState = false;

LED ledBuiltIn(LED_BUILTIN);

ADS1115_WE analogToDigitalConverter = ADS1115_WE(ADS1115_I2C_ADDRESS);

PID pidTrolley(TROLLEY_MOTOR_KP, TROLLEY_MOTOR_KI, TROLLEY_MOTOR_KD, TROLLEY_MOTOR_MAX_INTEGRAL, TROLLEY_MOTOR_MAX_OUTPUT);

#endif // VARIABLE_H