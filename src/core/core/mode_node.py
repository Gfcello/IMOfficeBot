import rclpy
from rclpy.node import Node

from message_types.msg import ControllerMode, CoreMode, GpsCoords, SystemStatus
from std_msgs.msg import Bool

from math import cos, pi, sin


class ModeNode(Node):
    def __init__(self):
        super().__init__('mode_node')

        self.mode = CoreMode.STARTUP_MODE

        timer_period = 0.5  # seconds # TODO: tune this
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Check the current mode
        loop_mode = self.mode
        match self.mode:
            case CoreMode.STARTUP_MODE:
                self.startup_loop()
            case CoreMode.IDLE_MODE:
                self.idle_loop()
            case CoreMode.GPS_TRACKING_MODE:
                self.gps_track_loop()
            case CoreMode.SEARCH_PATTERN_MODE:
                self.search_pattern_loop()
            case CoreMode.CAMERA_TRACKING_MODE:
                self.camera_tracking_loop()
            case CoreMode.AT_TARGET_MODE:
                self.at_target_loop()
            case CoreMode.TELEOP_MODE:
                self.teleop_loop()
            case CoreMode.ERROR_MODE:
                self.error_loop()

        # Log mode changes
        if loop_mode != self.mode:
            self.get_logger().info(f'Mode changed from {loop_mode} to {self.mode}.')

            # Publish Current mode to be sent to GUI when mode changes
            self.mode_msg.mode = self.mode
            self.mode_publisher.publish(self.mode_msg)

    def status_callback(self, msg):
        # Update system status
        self.system_status_array = msg.node_statuses

    def check_no_apocalypse(self):
        # Check to see if the bot needs to go into error mode
        # True means no apocalypse statuses
        return max(self.system_status_array) < SystemStatus.STATUS_APOCALYPSE

    def check_sensors_started(self):
        return (self.system_status_array[SystemStatus.DRIVING_NODE] > SystemStatus.STATUS_IDLE)
        # Add more as more sensors get added

    def set_controller_mode(self, controller_mode):
        controller_msg = ControllerMode()
        controller_msg.mode = controller_mode
        self.controller_mode_publisher.publish(controller_msg)

    def startup_loop(self):
        # Mode for startup

        # Check mode switch conditions
        if self.check_no_apocalypse():
            # Check sensor nodes have started
            if self.check_sensors_started():
                # Anything to do when exiting the startup mode

                self.mode = CoreMode.IDLE_MODE
        else:
            # Anything to do in case of error
            self.mode = CoreMode.ERROR_MODE

    def idle_loop(self):
        # Mode for waiting before mode is selected

        # Check mode switch conditions
        if self.check_no_apocalypse():
            # Add different mode change conditions here
            pass
        else:
            # Anything to do in case of error
            self.mode = CoreMode.ERROR_MODE

    def error_loop(self):
        # Mode for errors
        pass
        # No switch conditions, there is no escape yet


def main(args=None):
    rclpy.init(args=args)

    mode_node = ModeNode()

    rclpy.spin(mode_node)


if __name__ == '__main__':
    main()
