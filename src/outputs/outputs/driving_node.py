import rclpy
from rclpy.node import Node
import smbus

from pca9685_driver import Device

from message_types.msg import NodeStatus, DriveCmd
from std_msgs.msg import Bool


class DrivingNode(Node):
    # Communicates with PCA 9685 PWM generator over I2C to control 2 continuous rotation servo motors
    # Requires:
    # sudo apt-get install build-essential libi2c-dev i2c-tools python-dev libffi-dev
    # and
    # pip install PCA9685-driver

    FREQ = 50
    def __init__(self):
        super().__init__('driving_node')
        self.status_publisher = self.create_publisher(NodeStatus, 'driving_node/status', 10)
        # Subscribe to driving commands
        self.create_subscription(DriveCmd, 'controller_node/drive_cmd', self.update_motors, 10)
        self.create_subscription(Bool, 'mode_node/enable_motors', self.enable_motors, 10)

        self.status_msg = NodeStatus()
        self.declare_parameter('i2c_addr', 0x40)
        self.i2c_addr = self.get_parameter('i2c_addr').get_parameter_value().integer_value

        self.declare_parameter('left_port_num', 0)
        self.left_motor_port = self.get_parameter('left_port_num').get_parameter_value().integer_value
        self.declare_parameter('right_port_num', 3)
        self.right_motor_port = self.get_parameter('right_port_num').get_parameter_value().integer_value

        self.chip = Device(self.i2c_addr)
        self.chip.set_pwm_frequency(self.FREQ)

        self.status_msg.status = NodeStatus.STATUS_IDLE

        self.get_logger().info('Driving Node Started')

        timer_period = 1.0  # seconds, 1hz status updates
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Publish the most recent status
        self.status_publisher.publish(self.status_msg)
    
    def sleep_chip(self):
        self.status_msg.status = NodeStatus.STATUS_IDLE
        self.get_logger().info('Driving Disabled')
        self.chip.sleep()

    def wake_chip(self):
        self.status_msg.status = NodeStatus.STATUS_GOOD
        self.get_logger().info('Driving Enabled')
        self.chip.wake()
    
    def enable_motors(self, msg):
        if msg.data:
            self.wake_chip()
        else:
            self.sleep_chip()

    def update_motors(self, msg):
        self.left_throttle = msg.left_throttle
        self.right_throttle = msg.right_throttle

        # saturate throttle
        if self.left_throttle > 1.0:
            self.left_throttle = 1.0
        elif self.left_throttle < -1.0:
            self.left_throttle = -1.0
        
        if self.right_throttle > 1.0:
            self.right_throttle = 1.0
        elif self.right_throttle < -1.0:
            self.right_throttle = -1.0

        # map throttle (-1 to 1) -> 1-2ms / 20ms -> 0-4095
        # First convert -1 to 1 to between 1 and 2 ms:
        left_value = ((self.left_throttle) / 2) + 1.5
        right_value = ((-1 * self.right_throttle) / 2) + 1.5 # need to flip right motor dir
        # Next convert ms to duty:
        left_value /= 20.0
        right_value /= 20.0
        # finally convert duty to compare register value
        left_value = int(left_value * 4095.0)
        right_value = int(right_value * 4095.0)
        self.chip.set_pwm(self.left_motor_port, left_value)
        self.chip.set_pwm(self.right_motor_port, right_value)


def main(args=None):
    rclpy.init(args=args)

    driving_node = DrivingNode()

    rclpy.spin(driving_node)


if __name__ == '__main__':
    main()
