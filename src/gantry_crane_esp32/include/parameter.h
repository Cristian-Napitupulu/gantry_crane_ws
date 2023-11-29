#ifndef PARAMETER_H
#define PARAMETER_H

#define SERIAL_BAUDRATE 115200

#define NODE_NAME "microcontroller_gantry"
#define ENCODER_TOPIC_NAME "encoder_value"
#define TROLLEY_PWM_TOPIC_NAME "trolley_pwm"
#define HOIST_PWM_TOPIC_NAME "hoist_pwm"

#define ENCODER_PUBLISH_PERIOD_MS 10
#define ENCODER_PUBLISH_TIMEOUT_NS 20

#define TROLLEY_PWM_SUBSCRIBER_TIMEOUT_NS 20
#define HOIST_PWM_SUBSCRIBER_TIMEOUT_NS 20

// When choosing a pin, make sure it is not conflicting with other pins.
// For more information, see https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
#define ENCODER_CHANNEL_A_PIN 27
#define ENCODER_CHANNEL_B_PIN 26

#define TROLLEY_MOTOR_FORWARD_PIN 4
#define TROLLEY_MOTOR_REVERSE_PIN 5
#define TROLLEY_MOTOR_PWM_PIN 2

#define HOIST_MOTOR_FORWARD_PIN 32
#define HOIST_MOTOR_REVERSE_PIN 33
#define HOIST_MOTOR_PWM_PIN 34

#define PCA9685_ADDRESS 0x40

#define ADS1115_ADDRESS 0x48

#endif