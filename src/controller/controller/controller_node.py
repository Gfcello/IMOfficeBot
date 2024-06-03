import rclpy
from rclpy.node import Node

from message_types.msg import ControllerMode, DriveCmd, ImuData, NodeStatus, TeleopCmd
from std_msgs.msg import Bool, Float64

from math import atan, cos, pi, sin, sqrt


class ControllerNode(Node):
    DEBUG = False
    FREQ = 100

    def __init__(self):
        super().__init__('controller_node')

        # Read PID tuning parameters from configuration file
        self.declare_parameter('HEADING_KP', 1/90)
        self.HEADING_KP = self.get_parameter(
                'HEADING_KP').value

        self.declare_parameter('TURNING_KP', 0.05)
        self.TURNING_KP = self.get_parameter(
                'TURNING_KP').value

        # Publisher for motor throttles.
        self.drive_cmd_publisher = self.create_publisher(DriveCmd, 'controller_node/drive_cmd', 10)

        # Status Publisher
        self.status_publisher = self.create_publisher(NodeStatus, 'controller_node/status', 10)

        # A subscriber for the mode node's signal to change control modes
        self.create_subscription(ControllerMode, 'mode_node/controller_mode',
                                 self.change_mode_callback, 10)
        
        # A subscriber for teleop commands, not prefixed as from external script
        self.create_subscription(TeleopCmd, 'teleop_cmd',
                                 self.teleop_cmd_callback, 10)

        # Subscriber to IMU data to read the bot's heading and maybe use other data
        self.latest_imu_data = ImuData()
        self.create_subscription(ImuData, 'imu_node/data',
                                 self.imu_data_callback, 10)

        self.declare_parameter('TELEOP_FILTER_LEN', 5)
        self.TELEOP_FILTER_LEN = self.get_parameter(
                'TELEOP_FILTER_LEN').value

        self.mode = ControllerMode.IDLE
        self.mode_changed = False

        self.new_imu_data = False

        self.target_ang_vel = 0.0 # in deg / sec, likely go in 2 deg/s increments
        self.target_throttle = 0.0 # in terms of throttle that differential
        self.straight_path_heading = 0.0 # Used to drive in a straight line

        self.status_msg = NodeStatus()

        self.timer_period = 0.1  # seconds (10Hz.)
        self.create_timer(self.timer_period, self.timer_callback)

        status_timer_period = 1.0  # seconds (1Hz.)
        self.create_timer(status_timer_period, self.status_timer_callback)

    def timer_callback(self):
        if self.mode_changed:
            self.mode_changed = False
            # Reset any filters with memory

        match self.mode:
            case ControllerMode.TELEOP:
                self.teleop_loop()
            case _:
                # if no mode selected (start of operation or idle) then pass
                self.status_msg.status = NodeStatus.STATUS_IDLE

    def status_timer_callback(self):
        # Publish most recent status
        self.status_publisher.publish(self.status_msg)

    def change_mode_callback(self, msg):
        # Update the system status message at the node's index
        if self.mode != msg.mode:
            self.mode_changed = True
            self.mode = msg.mode
    
    def teleop_cmd_callback(self, msg):
        self.target_ang_vel = msg.cw_deg_per_s
        self.target_throttle = msg.fwd_throttle
        if self.target_ang_vel == 0:
            self.straight_path_heading = self.cur_heading

    def imu_data_callback(self, msg):
        self.cur_heading = msg.heading
        self.cur_ang_vel = msg.ang_vel

    def teleop_loop(self):
        # If driving straight then try to hold a heading, if turning then hold an angular velocity

        if self.target_ang_vel == 0.0:
            # handle jump from 0-360:
            if abs(self.straight_path_heading - self.cur_heading) > 180:
                if self.straight_path_heading > 180:
                    # Case where straight heading is large and cur heading is small
                    ang_error = (self.straight_path_heading - 360) - self.cur_heading
                else:
                    # Case where straight heading is small and cur heading is large
                    ang_error = self.straight_path_heading - (self.cur_heading - 360)
            else:
                ang_error = self.straight_path_heading - self.cur_heading

            left_throttle = self.target_throttle
            right_throttle = self.target_throttle
            left_throttle += self.HEADING_KP * ang_error
            right_throttle -= self.HEADING_KP * ang_error
        else:
            # Turning error is CW, so if positive then left needs to go faster
            ang_vel_error = self.target_ang_vel - self.cur_ang_vel
            left_throttle = self.target_throttle
            right_throttle = self.target_throttle
            left_throttle += self.TURNING_KP * ang_vel_error
            right_throttle -= self.TURNING_KP * ang_vel_error

        throttle_msg = DriveCmd()
        throttle_msg.left_throttle = left_throttle
        throttle_msg.right_throttle = right_throttle
        self.drive_cmd_publisher.publish(throttle_msg)
    
        self.status_msg.status = NodeStatus.STATUS_GOOD

def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)


if __name__ == '__main__':
    main()
