import rclpy
from rclpy.node import Node

import struct

from message_types.msg import MotionMsg, NodeStatus

# NOTES:
# Mouse driver should be disabled to not have it click or move by accident:
# Need to use x11 noy wayland for this to work (select xorg on login)
# 1. Find device id using $ xinput -list
# 2. Disable device driver $ xinput --disable <id>

# To re-enable device driver: $ xinput --enable <id>

# Can find mouse USB data using ls /dev, appears as /dev/input/mice for first wired mouse
# was able to CAT the data, but some characters not showing.

# https://www.eecg.utoronto.ca/~jayar/ece241_08F/AudioVideoCores/ps2/ps2.html#mousedata


class MouseNode(Node):
    PIX_PER_METER = 48500 # from calibration script

    right_motion = 0
    fwd_motion = 0
    def __init__(self):
        super().__init__('mouse_node')

        self.declare_parameter('mount_point', '/dev/input/mice')
        self.mount_point = self.get_parameter('mount_point').get_parameter_value().string_value

        self.motion_publisher = self.create_publisher(MotionMsg, 'mouse_node/motion', 10)
        self.motion_msg = MotionMsg()

        self.status_publisher = self.create_publisher(NodeStatus, 'mouse_node/status', 10)
        self.status_msg = NodeStatus()

        self.input_file = open(self.mount_point, "rb")

        publish_period = 0.5  # seconds
        self.create_timer(publish_period, self.publish_callback)
        timer_period = 0.01 # TODO, check if there is loss at 100Hz, update this
        self.create_timer(timer_period, self.timer_callback)

    def publish_callback(self):
            self.motion_msg.fwd = self.fwd_motion
            self.motion_msg.right = self.right_motion

            # reset the motion since publish
            self.fwd_motion = 0
            self.right_motion = 0

            self.motion_publisher.publish(self.motion_msg)

            self.status_msg.status = self.status_msg.STATUS_GOOD
            self.status_publisher.publish(self.status_msg)
            # TODO check for mouse sensor status

    def timer_callback(self):
        # Read mouse and update the motion since last publish
        buf = self.input_file.read(3)
        # buttons = buf[0] # Currently not using the buttons as inputs
        # bLeft = buttons & 0x1
        # bMiddle = ( buttons & 0x4 ) > 0
        # bRight = ( buttons & 0x2 ) > 0
        x,y = struct.unpack( "bb", buf[1:] )
        self.fwd_motion += y / self.PIX_PER_METER
        self.right_motion += x / self.PIX_PER_METER

def main(args=None):
    rclpy.init(args=args)

    mouse_node = MouseNode()

    rclpy.spin(mouse_node)


if __name__ == '__main__':
    main()
