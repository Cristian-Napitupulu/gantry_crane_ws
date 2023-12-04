#ifndef PARAMETER_H
#define PARAMETER_H

#define SERIAL_BAUDRATE 115200

#define NODE_NAME "microcontroller_gantry"
#define TROLLEY_POSITION_TOPIC_NAME "trolley_position"
#define LIMIT_SWITCH_TOPIC_NAME "limit_switch"
#define MOTOR_PWM_TOPIC_NAME "motor_pwm"

#define POSITION_PUBLISH_PERIOD_MS 3
#define POSITION_PUBLISH_TIMEOUT_NS 20

#define MOTOR_PWM_SUBSCRIBER_TIMEOUT_NS 20

#define LIMIT_SWITCH_PUBLISH_PERIOD_MS 5
#define LIMIT_SWITCH_PUBLISH_TIMEOUT_NS 20

#define TROLLEY_MOTOR_PWM_MAX 120
#define HOIST_MOTOR_PWM_MAX 200

#define ENCODER_MAX_VALUE 27000
#define ENCODER_MIN_VALUE 0

#define POSITION_MAX_VALUE 1.5
#define POSITION_MIN_VALUE 0

// When choosing a pin, make sure it is not conflicting with other pins.
// For more information, see https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
// Pin 0 and 1 are used for serial communication, so don't use them.
// Pin 6 to 11 are used for SPI communication, so don't use them.
// Pin 34 to 39 are input only, so don't use them.
#define ENCODER_CHANNEL_A_PIN 33
#define ENCODER_CHANNEL_B_PIN 32

#define TROLLEY_MOTOR_FORWARD_PIN 27
#define TROLLEY_MOTOR_REVERSE_PIN 26
#define TROLLEY_MOTOR_PWM_PIN 25

#define HOIST_MOTOR_FORWARD_PIN 12
#define HOIST_MOTOR_REVERSE_PIN 13
#define HOIST_MOTOR_PWM_PIN 14

#define LIMIT_SWITCH_ENCODER_SIDE_PIN 15
#define LIMIT_SWITCH_TROLLEY_MOTOR_SIDE_PIN 4

#define LIMIT_SWITCH_NONE_TRIGGERED 0x00
#define LIMIT_SWITCH_ENCODER_SIDE_TRIGGERED 0x20
#define LIMIT_SWITCH_TROLLEY_MOTOR_SIDE_TRIGGERED 0x40
#define LIMIT_SWITCH_BOTH_TRIGGERED 0x7F

#define COMMAND_BRAKE_MOTOR 0xFFFF

#define PCA9685_I2C_ADDRESS 0x40

#define ADS1115_I2C_ADDRESS 0x48

#endif