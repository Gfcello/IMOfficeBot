import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

from math import cos, sin, pi

from message_types.msg import ImuData, MotionMsg, NodeStatus

class OdomNode(Node):
    # Fuse IMU and mouse sensor to track and publish odometry
    def __init__(self):
        super().__init__('odom_node')

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.odom_msg = Odometry()

        self.status_publisher = self.create_publisher(NodeStatus, 'odom_node/status', 10)
        self.status_msg = NodeStatus()

        self.imu_subscriber = self.create_subscription(ImuData, 'imu_node/data',
                                                       self.imu_callback, 10)
        
        # Publish at 2Hz when motion messages are recieved
        self.motion_subscriber = self.create_subscription(MotionMsg, 'mouse_node/motion',
                                                          self.motion_callback, 10)


    def imu_callback(self, msg):
        pass

    def motion_callback(self, msg):
        fwd_vel = msg.fwd * 2 # in m/s
        right_vel = msg.right * 2 # in m/s

        # use heading to convert to fixed frame vel (set x as north, y as east)
        x_vel = fwd_vel * cos(self.heading * pi / 180) - right_vel * sin(self.heading * pi / 180)
        y_vel = fwd_vel * sin(self.heading * pi / 180) + right_vel * cos(self.heading * pi / 180)

        # positions
        self.odom_msg.pose.pose.position.x/y/z
        # orientation (in quaternion form)
        self.odom_msg.pose.pose.orientation
        # velocities
        self.odom_msg.twist.twist.linear.x = x_vel
        self.odom_msg.twist.twist.linear.y = y_vel
        self.odom_msg.twist.twist.linear.z = 0 # no flying allowed

        self.odom_msg.twist.twist.angular.x = 0 # no rolling
        self.odom_msg.twist.twist.angular.y = 0 # no pitching
        self.odom_msg.twist.twist.angular.y = ((self.heading - self.last_pub_heading) * 2) * pi / 180 # all yaw

        self.odom_publisher.publish(self.odom_msg)

        self.last_pub_heading = self.heading

def main(args=None):
    rclpy.init(args=args)

    odom_node = OdomNode()

    rclpy.spin(odom_node)


if __name__ == '__main__':
    main()
