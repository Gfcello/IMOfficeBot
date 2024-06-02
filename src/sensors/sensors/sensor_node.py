import rclpy
from rclpy.node import Node

import pigpio

from message_types.msg import NodeStatus
from std_msgs.msg import Float64


class SensorNode(Node):
    # Define enums
    I2C = 1
    SERIAL = 2
    SPI = 3

    def __init__(self, node_name, com_type, i2c_address=0, serial_port='', status_publish_period=1.0):
        super().__init__(node_name)

        # Initialize a status publisher
        self.status_publisher = self.create_publisher(NodeStatus, f'{node_name}/status', 10)
        self.status_msg = NodeStatus()

        # Initialize an interface class
        self.init_interface(com_type, i2c_address, serial_port)
        self.get_logger().debug(f'{node_name} Interface opened')

        self.create_timer(status_publish_period, self.status_callback)

    def status_callback(self):
        self.status_publisher.publish(self.status_msg)

    def init_interface(self, com_type, i2c_address, serial_port):
        if com_type == self.I2C:
            self.interface = i2cInterface(i2c_address)
        # elif com_type == self.SERIAL:
        #     self.interface = serialInterface(serial_port)
        # elif com_type == self.SPI:
        #     self.interface = spiInterface()
        else:
            return
    
    def read(self):
        return self.interface.read()
