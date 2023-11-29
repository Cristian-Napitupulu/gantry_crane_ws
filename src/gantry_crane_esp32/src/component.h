#ifndef COMPONENT_H
#define COMPONENT_H

#include <encoder.hpp>
#include <motor.hpp>

#include "parameter.h"

Motor trolleyMotor(TROLLEY_MOTOR_FORWARD_PIN, TROLLEY_MOTOR_REVERSE_PIN, TROLLEY_MOTOR_PWM_PIN);
Motor hoistMotor(HOIST_MOTOR_FORWARD_PIN, HOIST_MOTOR_REVERSE_PIN, HOIST_MOTOR_PWM_PIN);

Encoder encoderTrolley(ENCODER_CHANNEL_A_PIN, ENCODER_CHANNEL_B_PIN);
void encoderTrolleyCallback(){
  encoderTrolley.update();
}

#endif // COMPONENT_H