#ifndef PARAMETER_H
#define PARAMETER_H

// Serial communication
#define SERIAL_BAUDRATE 115200

// micro-ROS node and topic names
#define NODE_NAME "microcontroller_gantry"
#define TROLLEY_POSITION_TOPIC_NAME "trolley_position"
#define LIMIT_SWITCH_TOPIC_NAME "limit_switch"
#define MOTOR_PWM_TOPIC_NAME "motor_pwm"
#define TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME "trolley_motor_voltage"
#define HOIST_MOTOR_VOLTAGE_TOPIC_NAME "hoist_motor_voltage"

// micro-ROS executor timeout
// trolley position publisher
#define POSITION_PUBLISH_PERIOD_MS 10
#define POSITION_PUBLISH_TIMEOUT_MS 12

// limit switch publisher
#define LIMIT_SWITCH_PUBLISH_PERIOD_MS 5
#define LIMIT_SWITCH_PUBLISH_TIMEOUT_MS 7

// trolley motor voltage publisher
#define TROLLEY_MOTOR_VOLTAGE_PUBLISH_PERIOD_MS 10
#define TROLLEY_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS 12

// hoist motor voltage publisher
#define HOIST_MOTOR_VOLTAGE_PUBLISH_PERIOD_MS 10
#define HOIST_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS 12

// motor PWM subscriber
#define MOTOR_PWM_SUBSCRIBER_TIMEOUT_MS 10

/*Minimum and maximum PWM values
* Minimum PWM value is the value when the motor is barely moving (dead zone)
* All values (absolute) below the minimum PWM value but above 0 will be set to minimum PWM value accordingly
* To stop the motor, set the PWM value to 0

* Maximum PWM value is the value when the motor is moving at maximum speed
* All values above the maximum PWM value will be set to maximum PWM value
* Used for protection 
*/

// Maximum and minimum PWM values for trolley motor
#define TROLLEY_MOTOR_PWM_MAX 150
#define TROLLEY_MOTOR_PWM_MIN 70

// Maximum and minimum PWM values for hoist motor
#define HOIST_MOTOR_PWM_MAX 255
#define HOIST_MOTOR_PWM_MIN 0

// Encoder
// Got from measurement
#define ENCODER_MAX_VALUE 27000
#define ENCODER_MIN_VALUE 0
#define POSITION_MAX_VALUE 1.5
#define POSITION_MIN_VALUE 0

/*Pin definitions 
* When choosing a pin, make sure it is not conflicting with other pins.
* For more information, see https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
* Pin 0 and 1 are used for serial communication, so don't use them.
* Pin 2 is used for notification LED, so don't use it.
* Pin 6 to 11 are used for SPI communication, so don't use them.
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
#define HOIST_MOTOR_FORWARD_PIN 12
#define HOIST_MOTOR_REVERSE_PIN 13
#define HOIST_MOTOR_PWM_PIN 14

// Limit switches pins
#define LIMIT_SWITCH_ENCODER_SIDE_PIN 15
#define LIMIT_SWITCH_TROLLEY_MOTOR_SIDE_PIN 4

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

// Command for brake motor
#define COMMAND_BRAKE_MOTOR 0xFFFF

// Mode
#define IDLE_MODE 0x00
#define MOVE_TO_ORIGIN_MODE 0x1F
#define MOVE_TO_MIDDLE_MODE 0x2F
#define MOVE_TO_END_MODE 0x3F
#define LOCK_CONTAINER_MODE 0x4F
#define UNLOCK_CONTAINER_MODE 0x5F
#define COLLECT_DATA_MODE 0xFF


// Calculation constants
#define FULL_BRIDGE_RECTIFIER_DIODE_VOLTAGE_DROP 0.7
#define VOLTAGE_DIVIDER_RESISTOR_RATIO 0.1686824765832

#endif