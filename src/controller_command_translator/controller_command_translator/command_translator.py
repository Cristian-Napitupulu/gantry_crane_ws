import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import UInt32
from std_msgs.msg import Int32

COMMAND_TRANSLATOR_NODE_NAME = "command_translator_node"

CONTROLLER_COMMAND_TOPIC_NAME = "controller_command"
TROLLEY_MOTOR_PWM_TOPIC_NAME = "trolley_motor_PWM"
HOIST_MOTOR_PWM_TOPIC_NAME = "hoist_motor_PWM"

PUBLISH_PERIOD = 0.02  # second (50Hz)


class CommandTranslator(Node):
    def __init__(self):
        super().__init__(COMMAND_TRANSLATOR_NODE_NAME)

        self.controller_command_subscriber = self.create_subscription(
            UInt32, CONTROLLER_COMMAND_TOPIC_NAME, self.controller_command_callback, 10
        )

        self.trolley_motor_PWM_publisher = self.create_publisher(
            Int32, TROLLEY_MOTOR_PWM_TOPIC_NAME, 10
        )
        self.hoist_motor_PWM_publisher = self.create_publisher(
            Int32, HOIST_MOTOR_PWM_TOPIC_NAME, 10
        )

        self.gantry_mode = 0
        self.pwm_trolley = 0
        self.pwm_hoist = 0

        self.last_command_time = self.get_clock().now()

        self.publish_timer = self.create_timer(PUBLISH_PERIOD, self.timer_callback)

        self.get_logger().info("Command Translator Node has been started.")

    def timer_callback(self):
        self.get_logger().debug(
            "Publishing: pwm_trolley: %d, pwm_hoist: %d"
            % (self.pwm_trolley, self.pwm_hoist)
        )

        if self.get_clock().now() - self.last_command_time > Duration(seconds=0.05):
            self.pwm_trolley = 0
            self.pwm_hoist = 0

        trolley_motor_PWM_message = Int32()
        trolley_motor_PWM_message.data = self.pwm_trolley
        self.trolley_motor_PWM_publisher.publish(trolley_motor_PWM_message)

        hoist_motor_PWM_message = Int32()
        hoist_motor_PWM_message.data = self.pwm_hoist
        self.hoist_motor_PWM_publisher.publish(hoist_motor_PWM_message)

    def controller_command_callback(self, message):
        self.gantry_mode, self.pwm_trolley, self.pwm_hoist = self.unpack_values(
            message.data
        )
        self.last_command_time = self.get_clock().now()

    def unpack_values(self, packed_value):
        # Extract the values from the packed 32-bit integer
        gantry_mode = packed_value & 0xFF
        pwm_trolley = (packed_value >> 8) & 0xFFF
        pwm_hoist = (packed_value >> 20) & 0xFFF

        # Convert two's complement representation back to negative values
        if pwm_trolley & 0x800:
            pwm_trolley = pwm_trolley - 0xFFF - 1
        if pwm_hoist & 0x800:
            pwm_hoist = pwm_hoist - 0xFFF - 1

        # Clamp pwm_trolley between -1023 and 1023
        pwm_trolley = max(-1023, min(1023, pwm_trolley))

        # Clamp pwm_hoist between -1023 and 1023
        pwm_hoist = max(-1023, min(1023, pwm_hoist))

        return gantry_mode, pwm_trolley, pwm_hoist


def main(args=None):
    rclpy.init(args=args)

    command_translator = CommandTranslator()

    rclpy.spin(command_translator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_translator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
