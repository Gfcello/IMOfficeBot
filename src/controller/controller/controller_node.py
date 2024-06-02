import rclpy
from rclpy.node import Node

from message_types.msg import ControllerMode, FwCwCmd, GpsCoords, ImuData, NodeStatus
from std_msgs.msg import Bool, Float64

from math import atan, cos, pi, sin, sqrt


class ControllerNode(Node):
    DEBUG = False
    FREQ = 100

    def __init__(self):
        super().__init__('diagnostic_node')

        self.declare_parameter('GPS_TARGET_TOL', 0.0005)
        self.GPS_TARGET_TOL = self.get_parameter(
                'GPS_TARGET_TOL').value

        # Read PID tuning parameters from configuration file
        self.declare_parameter('GPS_KP_LIN', 0.0)
        self.GPS_KP_LIN = self.get_parameter('GPS_KP_LIN').value

        self.declare_parameter('GPS_KP_ANG', 0.0)
        self.GPS_KP_ANG = self.get_parameter('GPS_KP_ANG').value

        self.declare_parameter('TELEOP_KP_ANG', 0.0)
        self.TELEOP_KP_ANG = self.get_parameter(
                'TELEOP_KP_ANG').value

        self.declare_parameter('PID_HISTORY_LEN', 100)
        self.PID_HISTORY_LEN = self.get_parameter(
                'PID_HISTORY_LEN').value

        # Angular error over which the robot turns in place
        self.declare_parameter('TURN_IN_PLACE_ANG', pi / 4)
        self.TURN_IN_PLACE_ANG = self.get_parameter(
                'TURN_IN_PLACE_ANG').value

        self.declare_parameter('CAM_FILTER_LEN', 5)
        self.CAM_FILTER_LEN = self.get_parameter(
                'CAM_FILTER_LEN').value

        # Publishers for each thruster throttle.
        self.thruster_1_throttle_publisher = self.create_publisher(Float64,
                                                                   'thruster_1_node/throttle', 10)
        self.thruster_2_throttle_publisher = self.create_publisher(Float64,
                                                                   'thruster_2_node/throttle', 10)

        # Publisher to let the mode node know when the bot reaches the target
        self.at_gps_target_publisher = self.create_publisher(Bool,
                                                             'controller_node/at_gps_target', 10)

        # Status Publisher
        self.status_publisher = self.create_publisher(NodeStatus, 'controller_node/status', 10)

        # A subscriber for the mode node's signal to change control modes
        self.create_subscription(ControllerMode, 'mode_node/controller_mode',
                                 self.change_mode_callback, 10)

        # A subscriber for the GPS target from the mode node
        self.latest_gps_data = GpsCoords()
        self.create_subscription(GpsCoords, 'mode_node/gps_target',
                                 self.set_gps_target_callback, 10)

        # Subscriber to IMU data to read the bot's heading and maybe use other data
        self.latest_imu_data = ImuData()
        self.create_subscription(ImuData, 'imu_node/data',
                                 self.imu_data_callback, 10)

        # Subscriber to GPS data
        self.latest_gps_data = GpsCoords()
        self.create_subscription(GpsCoords, 'gps_node/gps_coords',
                                 self.gps_data_callback, 10)

        # Subscriber to teleop data
        self.teleop_cmds = []
        self.create_subscription(FwCwCmd, 'radio_node/receive/teleop_cmd',
                                 self.teleop_cmd_callback, 10)

        # Subscriber to camera target
        self.camera_cmds = []
        self.create_subscription(FwCwCmd, 'camera_fuse_node/camera_drive_target',
                                 self.camera_cmd_callback, 10)

        self.declare_parameter('TELEOP_FILTER_LEN', 5)
        self.TELEOP_FILTER_LEN = self.get_parameter(
                'TELEOP_FILTER_LEN').value

        self.mode = ControllerMode.IDLE
        self.mode_changed = False

        self.testing = False
        if self.testing:
            self.mode = ControllerMode.TELEOP
            self.gps_target = GpsCoords()
            self.gps_target.latitude = 43.0
            self.gps_target.longitude = 79.0
        else:
            self.gps_target = None

        self.new_gps_data = False
        self.new_imu_data = False

        self.teleop_line_path = False
        self.straight_path_heading = 0

        self.gps_pos_approx = GpsCoords()

        self.declare_parameter('IMU_FILTER_LEN', 10)
        self.IMU_FILTER_LEN = self.get_parameter('IMU_FILTER_LEN').value

        self.declare_parameter('GPS_FILTER_LEN', 10)
        self.GPS_FILTER_LEN = self.get_parameter('GPS_FILTER_LEN').value

        # Assuming starting still
        self.imu_x_vel = 0
        self.imu_z_ang_vel = 0
        self.imu_x_vel_list = []
        self.imu_z_ang_vel_list = []
        self.IMU_DATA_PERIOD = 0.01  # IMU running at 100Hz.

        self.gps_right_throttle_filter = [0] * self.GPS_FILTER_LEN
        self.gps_left_throttle_filter = [0] * self.GPS_FILTER_LEN

        self.status_msg = NodeStatus()

        self.timer_period = 0.01  # seconds (100Hz.)
        self.create_timer(self.timer_period, self.timer_callback)

        status_timer_period = 0.5  # seconds (2Hz.)
        self.create_timer(status_timer_period, self.status_timer_callback)

    def timer_callback(self):
        if self.mode_changed:
            self.mode_changed = False
            # Reset low pass filters for teleop and camera
            self.teleop_cmds = []
            self.camera_cmds = []
            self.gps_right_throttle_filter = [0] * self.GPS_FILTER_LEN
            self.gps_left_throttle_filter = [0] * self.GPS_FILTER_LEN

        match self.mode:
            case ControllerMode.GPS_TRACK:
                self.gps_track_loop()
            case ControllerMode.CAMERA_TRACK:
                self.camera_track_loop()
            case ControllerMode.TELEOP:
                self.teleop_loop()
            case _:
                # if no mode selected (start of operation) then pass
                pass

    def status_timer_callback(self):
        # Publish most recent status
        self.status_publisher.publish(self.status_msg)

    def change_mode_callback(self, msg):
        # Update the system status message at the node's index
        if self.mode != msg.mode:
            self.mode_changed = True
            self.mode = msg.mode

    def set_gps_target_callback(self, msg):
        self.gps_target = msg
        self.gps_right_throttle_filter = [0] * self.GPS_FILTER_LEN
        self.gps_left_throttle_filter = [0] * self.GPS_FILTER_LEN

    def camera_cmd_callback(self, msg):
        # Keep last few messages
        self.camera_cmds.append(msg)
        if len(self.camera_cmds) > self.CAMERA_FILTER_LEN:
            self.camera_cmds.pop(0)

    def teleop_cmd_callback(self, msg):
        # Keep last few messages
        self.teleop_cmds.append([msg.fwd_cmd, msg.cw_cmd])
        if len(self.teleop_cmds) > self.TELEOP_FILTER_LEN:
            self.teleop_cmds.pop(0)

    def imu_data_callback(self, msg):
        self.new_imu_data = True
        self.latest_imu_data = msg
        # Low pass filter the accel and gyro velocity
        self.imu_x_vel_list.append(msg.accel_x * self.IMU_DATA_PERIOD)
        if len(self.imu_x_vel_list) > self.IMU_FILTER_LEN:
            self.imu_x_vel_list.pop(0)
        self.imu_z_ang_vel_list.append(msg.gyro_z * self.IMU_DATA_PERIOD)
        if len(self.imu_z_ang_vel_list) > self.IMU_FILTER_LEN:
            self.imu_z_ang_vel_list.pop(0)
        self.imu_x_vel += sum(self.imu_x_vel_list) * self.IMU_DATA_PERIOD / self.IMU_FILTER_LEN
        self.imu_z_ang_vel += (sum(self.imu_z_ang_vel_list) * self.IMU_DATA_PERIOD
                               / self.IMU_FILTER_LEN)
        self.imu_heading = msg.mag_z

    def gps_data_callback(self, msg):
        self.new_gps_data = True
        self.latest_gps_data = msg
        # Update the approximated position to the GPS position
        self.gps_pos_approx = msg

    def meters_to_lat(self, meters):
        # Calculation from Wikipedia
        # https://en.wikipedia.org/wiki/Geographic_coordinate_system#Latitude_and_longitude
        # Use last GPS message for the latitude range
        lat = self.latest_gps_data.latitude
        m_per_deg = (111132.92 - 559.82 * cos(2 * lat) + 1.175 * cos(4 * lat)
                     - 0.0023 * cos(6 * lat))
        return meters / m_per_deg

    def meters_to_long(self, meters):
        # Calculation from Wikipedia:
        # https://en.wikipedia.org/wiki/Geographic_coordinate_system#Latitude_and_longitude
        # Use last GPS message for the latitude range
        long = self.latest_gps_data.longitude
        m_per_deg = 111412.84 * cos(long) - 93.5 * cos(3 * long) + 0.118 * cos(5 * long)
        return meters / m_per_deg

    def gps_track_loop(self):
        # Get the error using gps and IMU
        linear_error = False
        angular_error = 1000
        if self.new_gps_data:  # Calculate positional error with GPS measurement
            self.new_gps_data = False

            if self.DEBUG:
                self.get_logger().info('GPS Data Case')
            
            lat_error = self.gps_target.latitude - self.latest_gps_data.latitude
            long_error = self.gps_target.longitude - self.latest_gps_data.longitude

            linear_error = sqrt(lat_error**2 + long_error**2)

            # Angular error is target - current heading
            angular_error = atan(long_error / lat_error) - self.latest_imu_data.mag_z

        elif self.new_imu_data:  # If no new GPS data approximate using last GPS and IMU
            self.new_imu_data = False
            if self.DEBUG:
                self.get_logger().info('IMU Data Case')

            # First approximate the position change with velocity and heading
            lat_increment = self.imu_x_vel * cos(self.latest_imu_data.mag_z) * self.timer_period
            long_increment = self.imu_x_vel * sin(self.latest_imu_data.mag_z) * self.timer_period

            # Convert from meters to degrees (Latitude changes meters per degree)
            lat_increment = self.meters_to_lat(lat_increment)
            long_increment = self.meters_to_long(long_increment)

            self.gps_pos_approx.latitude += lat_increment
            self.gps_pos_approx.longitude += long_increment

            lat_error = self.gps_target.latitude - self.gps_pos_approx.latitude
            long_error = self.gps_target.longitude - self.gps_pos_approx.longitude

            linear_error = sqrt(lat_error**2 + long_error**2)

            # Angular error is target - current heading
            angular_error = atan(long_error / lat_error) - self.latest_imu_data.mag_z

        if linear_error and abs(linear_error) < self.GPS_TARGET_TOL:
            # At target case
            if self.DEBUG:
                self.get_logger().info('Within Tolerance')
            # Close to GPS location, publish that
            self.at_gps_target_publisher.publish(Bool(data=True))

            # Stop and wait for either a new target or to change modes
            thruster_1_msg = Float64(data=0.0)
            thruster_2_msg = Float64(data=0.0)

            self.thruster_1_throttle_publisher.publish(thruster_1_msg)
            self.thruster_2_throttle_publisher.publish(thruster_2_msg)

            self.status_msg.status = NodeStatus.STATUS_IDLE

        elif angular_error != 1000 and angular_error > self.TURN_IN_PLACE_ANG:
            # Turn in place case
            if self.DEBUG:
                self.get_logger().info(f'Turn In Place Ang Error: {angular_error}')
            # In here check if the angular error is > TURN_IN_PLACE_ANG_ERROR
            # if so then don't run the linear controller, just turn in place
            ang_controller_effort = self.GPS_KP_ANG * angular_error

            if self.DEBUG:
                self.get_logger().info(f'Ang P Out: {ang_controller_effort}')

            left_thruster_throttle = ang_controller_effort
            right_thruster_throttle = -1.0 * ang_controller_effort

            if self.DEBUG:
                self.get_logger().info(f'Left Thruster Out: {left_thruster_throttle}')
                self.get_logger().info(f'Right Thruster Out: {right_thruster_throttle}')

            # And saturate to between +/- 0.8
            if left_thruster_throttle > 0.8:
                left_thruster_throttle = 0.8
            elif left_thruster_throttle < -0.8:
                left_thruster_throttle = -0.8

            if right_thruster_throttle > 0.8:
                right_thruster_throttle = 0.8
            elif right_thruster_throttle < -0.8:
                right_thruster_throttle = -0.8

            self.gps_left_throttle_filter.append(left_thruster_throttle)
            self.gps_left_throttle_filter.pop(0)

            self.gps_right_throttle_filter.append(left_thruster_throttle)
            self.gps_right_throttle_filter.pop(0)

            right_thruster_throttle = (sum(self.gps_right_throttle_filter)
                                       / len(self.gps_right_throttle_filter))
            left_thruster_throttle = (sum(self.gps_left_throttle_filter)
                                      / len(self.gps_left_throttle_filter))

            thruster_1_msg = Float64(data=left_thruster_throttle)
            thruster_2_msg = Float64(data=right_thruster_throttle)

            self.thruster_1_throttle_publisher.publish(thruster_1_msg)
            self.thruster_2_throttle_publisher.publish(thruster_2_msg)

            self.status_msg.status = NodeStatus.STATUS_GOOD

        elif angular_error != 1000 and linear_error:
            # Full Motion Case
            if self.DEBUG:
                self.get_logger().info(f'Ang Error: {angular_error}')
                self.get_logger().info(f'Lin Error: {linear_error}')

            ang_controller_effort = self.GPS_KP_ANG * angular_error
            lin_controller_effort = self.GPS_KP_LIN * linear_error

            if self.DEBUG:
                self.get_logger().info(f'Ang P Out: {ang_controller_effort}')
                self.get_logger().info(f'Lin P Out: {lin_controller_effort}')

            # Saturate linear effort to ensure that turning has an effect
            if lin_controller_effort > 0.8:
                lin_controller_effort = 0.8
            elif lin_controller_effort < -0.8:
                lin_controller_effort = -0.8

            # Saturate angular effort as > 0.6 doesn't improve turning
            if ang_controller_effort > 0.6:
                ang_controller_effort = 0.6
            elif ang_controller_effort < -0.6:
                ang_controller_effort = -0.6

            # Combine linear and angular controllers for thruster inputs
            left_thruster_throttle = lin_controller_effort + ang_controller_effort
            right_thruster_throttle = lin_controller_effort - ang_controller_effort

            self.gps_left_throttle_filter.append(left_thruster_throttle)
            self.gps_left_throttle_filter.pop(0)

            self.gps_right_throttle_filter.append(left_thruster_throttle)
            self.gps_right_throttle_filter.pop(0)

            right_thruster_throttle = (sum(self.gps_right_throttle_filter)
                                       / len(self.gps_right_throttle_filter))
            left_thruster_throttle = (sum(self.gps_left_throttle_filter)
                                      / len(self.gps_left_throttle_filter))

            if self.DEBUG:
                self.get_logger().info(f'Left Thruster Out: {left_thruster_throttle}')
                self.get_logger().info(f'Right Thruster Out: {right_thruster_throttle}')

            # And saturate to between +/- 0.8
            if left_thruster_throttle > 0.8:
                left_thruster_throttle = 0.8
            elif left_thruster_throttle < -0.8:
                left_thruster_throttle = -0.8

            if right_thruster_throttle > 0.8:
                right_thruster_throttle = 0.8
            elif right_thruster_throttle < -0.8:
                right_thruster_throttle = -0.8

            thruster_1_msg = Float64(data=left_thruster_throttle)
            thruster_2_msg = Float64(data=right_thruster_throttle)

            self.thruster_1_throttle_publisher.publish(thruster_1_msg)
            self.thruster_2_throttle_publisher.publish(thruster_2_msg)

            self.status_msg.status = NodeStatus.STATUS_GOOD

    def camera_track_loop(self):
        # Use a low pass filter to smooth sharp inputs
        fwd_error = 0.0
        cw_error = 0.0

        if len(self.camera_cmds) > 0:
            # Low pass filter the camera cmd input to smooth camera feed
            for cmd in self.camera_cmds:
                fwd_error += cmd.fwd_cmd
                cw_error += cmd.cw_cmd
            fwd_error /= len(self.camera_cmds)
            cw_error /= len(self.camera_cmds)

            if self.DEBUG:
                self.get_logger().info(f'Cam Ang Error: {cw_error}')
                self.get_logger().info(f'Cam Lin Error: {fwd_error}')

            left_thruster_throttle = fwd_error + cw_error
            right_thruster_throttle = fwd_error - cw_error

            # And saturate to between +/- 0.8
            if left_thruster_throttle > 0.8:
                left_thruster_throttle = 0.8
            elif left_thruster_throttle < -0.8:
                left_thruster_throttle = -0.8

            if right_thruster_throttle > 0.8:
                right_thruster_throttle = 0.8
            elif right_thruster_throttle < -0.8:
                right_thruster_throttle = -0.8

            thruster_1_msg = Float64(data=left_thruster_throttle)
            thruster_2_msg = Float64(data=right_thruster_throttle)

            self.thruster_1_throttle_publisher.publish(thruster_1_msg)
            self.thruster_2_throttle_publisher.publish(thruster_2_msg)

            self.status_msg.status = NodeStatus.STATUS_GOOD

        else:
            self.status_msg.status = NodeStatus.STATUS_IDLE

    def teleop_loop(self):
        # Use a low pass filter to smooth sharp inputs
        fwd_throttle = 0.0
        cw_throttle = 0.0

        if len(self.teleop_cmds):
            for cmd in self.teleop_cmds:
                fwd_throttle += cmd[0]
                cw_throttle += cmd[1]
            fwd_throttle /= len(self.teleop_cmds)
            cw_throttle /= len(self.teleop_cmds)

            if fwd_throttle > 0.8:
                fwd_throttle = 0.8
            elif fwd_throttle < -0.8:
                fwd_throttle = -0.8

            if cw_throttle > 0.6:
                cw_throttle = 0.6
            elif cw_throttle < -0.6:
                cw_throttle = -0.6

            if self.DEBUG:
                self.get_logger().info(f'Teleop Ang: {cw_throttle}')
                self.get_logger().info(f'Teleop lin: {fwd_throttle}')

            if cw_throttle == 0 and fwd_throttle != 0 and not self.teleop_line_path:
                self.teleop_line_path = True
                if self.DEBUG:
                    self.get_logger().info(f'Hold Heading: {self.imu_heading}')
                self.straight_path_heading = self.imu_heading

            elif cw_throttle != 0 or fwd_throttle == 0:
                # Turn off heading lock drift correction
                self.teleop_line_path = False

            left_thruster_throttle = 0
            right_thruster_throttle = 0

            if self.teleop_line_path:
                # Correct for deviations from current heading
                drift_correction = self.TELEOP_KP_ANG * (self.straight_path_heading
                                                         - self.imu_heading)

                if self.DEBUG:
                    self.get_logger().info(f'Drift Correction: {drift_correction}')

                left_thruster_throttle = fwd_throttle + cw_throttle + drift_correction
                right_thruster_throttle = fwd_throttle - cw_throttle - drift_correction
            else:
                left_thruster_throttle = fwd_throttle + cw_throttle
                right_thruster_throttle = fwd_throttle - cw_throttle

            # And saturate to between +/- 0.8
            if left_thruster_throttle > 0.8:
                left_thruster_throttle = 0.8
            elif left_thruster_throttle < -0.8:
                left_thruster_throttle = -0.8

            if right_thruster_throttle > 0.8:
                right_thruster_throttle = 0.8
            elif right_thruster_throttle < -0.8:
                right_thruster_throttle = -0.8

            thruster_1_msg = Float64(data=left_thruster_throttle)
            thruster_2_msg = Float64(data=right_thruster_throttle)

            self.thruster_1_throttle_publisher.publish(thruster_1_msg)
            self.thruster_2_throttle_publisher.publish(thruster_2_msg)

            self.status_msg.status = NodeStatus.STATUS_GOOD
        else:
            self.status_msg.status = NodeStatus.STATUS_IDLE


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)


if __name__ == '__main__':
    main()
