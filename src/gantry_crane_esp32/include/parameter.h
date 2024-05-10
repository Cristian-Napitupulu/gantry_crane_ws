#ifndef PARAMETER_H
#define PARAMETER_H

// Serial communication
#define SERIAL_BAUDRATE 115200

// micro-ROS node and topic names
#define NODE_NAME "microcontroller_gantry"
#define TROLLEY_POSITION_TOPIC_NAME "trolley_position"
#define LIMIT_SWITCH_TOPIC_NAME "limit_switch"
#define CONTROLLER_COMMAND_TOPIC_NAME "controller_command"
#define TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME "trolley_motor_voltage"
#define HOIST_MOTOR_VOLTAGE_TOPIC_NAME "hoist_motor_voltage"

// micro-ROS executor timeout
// trolley position publisher
#define POSITION_PUBLISH_PERIOD_MS 3
#define POSITION_PUBLISH_TIMEOUT_MS 10

// limit switch publisher
#define LIMIT_SWITCH_PUBLISH_PERIOD_MS 3
#define LIMIT_SWITCH_PUBLISH_TIMEOUT_MS 10

// trolley motor voltage publisher
#define TROLLEY_MOTOR_VOLTAGE_PUBLISH_PERIOD_MS 3
#define TROLLEY_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS 10

// hoist motor voltage publisher
#define HOIST_MOTOR_VOLTAGE_PUBLISH_PERIOD_MS 3
#define HOIST_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS 10

// Controller Command subscriber
#define CONTROLLER_COMMAND_SUBSCRIBER_TIMEOUT_MS 10

// Timeout for controller command
#define CONTROLLER_COMMAND_TIMEOUT_MS 5

/*Minimum and maximum PWM values
* Minimum PWM value is the value when the motor is barely moving (dead zone)
* All values (absolute) below the minimum PWM value but above 0 will be set to minimum PWM value accordingly
* To stop the motor, set the PWM value to 0

* Maximum PWM value is the value when the motor is moving at maximum speed
* All values above the maximum PWM value will be set to maximum PWM value
* Used for protection 
*/

#define TROLLEY_MAX_SPEED 0.3   // m/s

// Operating PWM values for trolley motor`
#define TROLLEY_MOTOR_PWM_MAX 800
#define TROLLEY_MOTOR_PWM_MIN 0

// Operating PWM values for hoist motor
#define HOIST_MOTOR_PWM_MAX 1023
#define HOIST_MOTOR_PWM_MIN 0

#define TROLLEY_MOTOR_FIND_ORIGIN_PWM 7

// Encoder
// Got from measurement
#define ENCODER_MAX_VALUE 27000
#define ENCODER_MIN_VALUE 0
#define POSITION_MAX_VALUE 1.5
#define POSITION_MIN_VALUE 0

// #define TROLLEY_MOTOR_VOLTAGE_MOVING_AVERAGE_BUFFER_SIZE 5
// #define HOIST_MOTOR_VOLTAGE_MOVING_AVERAGE_BUFFER_SIZE 5

/*Pin definitions
* When choosing a pin, make sure it is not conflicting with other pins.
* For more information, see https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
* Pin 0 and 1 are used for serial communication, so don't use them.
* Pin 2 is used for notification LED, so don't use it.
* Pin 6 to 11 are used for SPI communication, so don't use them.
* Pin 21 to 23 are used for I2C communication, so don't use them.
* Pin 34 to 39 are input only, be careful when using them.
*/

// Encoder pins
#define ENCODER_CHANNEL_A_PIN 33
#define ENCODER_CHANNEL_B_PIN 32

// Troley motor pins
#define TROLLEY_MOTOR_FORWARD_PIN 27
#define TROLLEY_MOTOR_REVERSE_PIN 26
#define TROLLEY_MOTOR_PWM_PIN 25

// Hoist motor pins
#define HOIST_MOTOR_FORWARD_PIN 13
#define HOIST_MOTOR_REVERSE_PIN 19
#define HOIST_MOTOR_PWM_PIN 14

// Limit switches pins
#define LIMIT_SWITCH_ENCODER_SIDE_PIN 34
#define LIMIT_SWITCH_TROLLEY_MOTOR_SIDE_PIN 35


// I2C addressess
// PCA9685: Servo driver for locking mechanism
#define PCA9685_I2C_ADDRESS 0x40
// ADS1115: ADC for motor voltage measurement
#define ADS1115_I2C_ADDRESS 0x48

// Limit switch state
#define LIMIT_SWITCH_NONE_TRIGGERED 0x00
#define LIMIT_SWITCH_ENCODER_SIDE_TRIGGERED 0x2F
#define LIMIT_SWITCH_TROLLEY_MOTOR_SIDE_TRIGGERED 0x4F
#define LIMIT_SWITCH_BOTH_TRIGGERED 0x6F

// Mode
#define IDLE_MODE 0x0F
#define MOVE_TO_ORIGIN_MODE 0x1F
#define MOVE_TO_MIDDLE_MODE 0x2F
#define MOVE_TO_END_MODE 0x3F
#define LOCK_CONTAINER_MODE 0x4F
#define UNLOCK_CONTAINER_MODE 0x5F
#define CONTROL_MODE 0x6F

#define BRAKE_COMMAND 0x7FF

#endif