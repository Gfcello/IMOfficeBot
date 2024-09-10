import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from math import cos, sin, pi
import numpy as np

from message_types.msg import ImuData, MotionMsg, NodeStatus
from geometry_msgs.msg import TransformStamped

# Quaternion conversion
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class OdomNode(Node):
    # Cumulative positions (using IMU and mouse sensor motions)
    x = 0.0
    y = 0.0
    heading = 0.0

    # Fuse IMU and mouse sensor to track and publish odometry
    def __init__(self):
        super().__init__('odom_node')

        self.broadcaster = TransformBroadcaster(self)

        self.status_publisher = self.create_publisher(NodeStatus, 'odom_node/status', 10)
        self.status_msg = NodeStatus()

        self.imu_subscriber = self.create_subscription(ImuData, 'imu_node/data',
                                                       self.imu_callback, 10)
        
        # Publish at 2Hz when motion messages are recieved
        self.motion_subscriber = self.create_subscription(MotionMsg, 'mouse_node/motion',
                                                          self.motion_callback, 10)


    def imu_callback(self, msg):
        # Store current absolute heading from IMU
        self.heading = msg.heading * (pi/180) # convert to radians

    def motion_callback(self, msg):
        # Read message from mouse sensor node and publish transform
        self.x += msg.fwd * cos(self.heading) + msg.right * sin(self.heading) # X is initial forwards direction
        self.y += -msg.fwd * sin(self.heading) + msg.right * cos(self.heading)

        fwd_vel = msg.fwd * 2 # in m/s
        right_vel = msg.right * 2 # in m/s

        # use heading to convert to fixed frame vel (set x as north, y as east)
        x_vel = fwd_vel * cos(self.heading * pi / 180) - right_vel * sin(self.heading * pi / 180)
        y_vel = fwd_vel * sin(self.heading * pi / 180) + right_vel * cos(self.heading * pi / 180)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0 # no flying

        # orientation (in quaternion form)
        q = quaternion_from_euler(0, 0, self.heading) # heading already converted to rad from N
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)

    odom_node = OdomNode()

    rclpy.spin(odom_node)


if __name__ == '__main__':
    main()
