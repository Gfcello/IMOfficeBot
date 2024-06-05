import rclpy
from rclpy.node import Node

from message_types.msg import ControllerMode, DriveCmd, ImuData, NodeStatus, TeleopCmd
from std_msgs.msg import Bool

import time


class ControllerNode(Node):
    DEBUG = False
    FREQ = 100

    def __init__(self):
        super().__init__('controller_node')

        # Read PID tuning parameters from configuration file
        self.declare_parameter('HEADING_KP', 1/90)
        self.HEADING_KP = self.get_parameter('HEADING_KP').value

        self.declare_parameter('TURNING_KP', 0.05)
        self.TURNING_KP = self.get_parameter('TURNING_KP').value

        self.declare_parameter('LEFT_MOT_STOP', 0.00)
        self.LEFT_MOT_STOP = self.get_parameter('LEFT_MOT_STOP').value
        self.declare_parameter('LEFT_MOT_FWD_MAX', 1.00)
        self.LEFT_MOT_FWD_MAX = self.get_parameter('LEFT_MOT_FWD_MAX').value
        self.declare_parameter('LEFT_MOT_BACK_MAX', -1.00)
        self.LEFT_MOT_BACK_MAX = self.get_parameter('LEFT_MOT_BACK_MAX').value

        self.declare_parameter('RIGHT_MOT_STOP', 0.00)
        self.RIGHT_MOT_STOP = self.get_parameter('RIGHT_MOT_STOP').value
        self.declare_parameter('RIGHT_MOT_FWD_MAX', -1.00)
        self.RIGHT_MOT_FWD_MAX = self.get_parameter('RIGHT_MOT_FWD_MAX').value
        self.declare_parameter('RIGHT_MOT_BACK_MAX', 1.00)
        self.RIGHT_MOT_BACK_MAX = self.get_parameter('RIGHT_MOT_BACK_MAX').value

        # Publisher for motor throttles.
        self.drive_cmd_publisher = self.create_publisher(DriveCmd, 'controller_node/drive_cmd', 10)
        self.drive_enable_publisher = self.create_publisher(Bool, 'mode_node/enable_motors', 10)

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
        self.loop_timer = self.create_timer(self.timer_period, self.timer_callback)

        status_timer_period = 1.0  # seconds (1Hz.)
        self.create_timer(status_timer_period, self.status_timer_callback)

    def timer_callback(self):
        if self.mode_changed:
            self.mode_changed = False
            # Reset any filters or things with memory
            self.calibration_max_accel = 0.0 # reset max measured acceleration

        match self.mode:
            case ControllerMode.TELEOP:
                self.teleop_loop()
            case ControllerMode.CALIBRATION:
                self.calibration_loop()
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
        self.x_accel = msg.x_accel
        if self.mode == ControllerMode.CALIBRATION:
            self.calibration_max_accel = max(self.calibration_max_accel, self.x_accel)
    
    def calibration_loop(self):
        # pause the control loop timer
        self.loop_timer.cancel()
        self.get_logger().info('Beginning Calibration')
        # Reset zero values
        self.LEFT_MOT_STOP = 0.0
        self.RIGHT_MOT_STOP = 0.0
        ang_error = 100
        # for simplicty define messages for enable/disable
        enable_msg = Bool()
        enable_msg.data = True
        disable_msg = Bool()
        disable_msg.data = False
        self.get_logger().info('Calibrating stop Angle')
        while abs(ang_error) > 3.0: # TODO: is 3 deg good for a tuning tol?
            # first calibrate stopped values by first setting both motors to 0.00
            drive_msg = DriveCmd()
            drive_msg.left_throttle = self.LEFT_MOT_STOP
            drive_msg.right_throttle = self.RIGHT_MOT_STOP
            self.drive_enable_publisher.publish(disable_msg) # Stop the bot for a bit to get the heading
            time.sleep(3.0)
            start_ang = self.cur_heading
            self.drive_enable_publisher.publish(enable_msg)
            self.drive_cmd_publisher.publish(drive_msg)
            # Wait for 3 seconds then see which way it is turning.
            time.sleep(3.0)
            self.drive_enable_publisher.publish(disable_msg) # Stop the bot
            self.get_logger().info(f'start_ang: {start_ang}, cur: {self.cur_heading}')
            ang_error = start_ang - self.cur_heading
            # Correct in case angle went from a small positive to ~ 360 or opposite
            if ang_error > 180:
                ang_error -= 360
            elif ang_error < -180:
                ang_error += 360
            # increase and decrease the wheel values until heading is not changing
            if ang_error > 3.0:
                self.LEFT_MOT_STOP -= 0.02
                self.RIGHT_MOT_STOP += 0.02
            elif ang_error < -3.0:
                self.LEFT_MOT_STOP += 0.02
                self.RIGHT_MOT_STOP -= 0.02

        # Now that the heading is within 3 deg after 3 seconds fix if the bot is still moving

        self.get_logger().info('Calibrating stop magnitude')
        self.calibration_max_accel = 100 # outrageous number to start loop
        while abs(self.calibration_max_accel) > 0.05: # TODO: test if this is a good tolerance
            self.calibration_max_accel = 0.0 # reset max measured acceleration
            # Fully stop the bot (sleep the chip), reset the max acceleration on IMU and 
            self.drive_enable_publisher.publish(disable_msg)
            time.sleep(3) # wait a few seconds for the bot to stop
            # then set both motors to the values found above.
            drive_msg = DriveCmd()
            drive_msg.left_throttle = self.LEFT_MOT_STOP
            drive_msg.right_throttle = self.RIGHT_MOT_STOP
            self.drive_enable_publisher.publish(enable_msg)
            self.drive_cmd_publisher.publish(drive_msg)
            time.sleep(3) # wait for bot to maybe drive a bit
            self.drive_enable_publisher.publish(disable_msg)

            # If acceleration is fwd/back then adjust both motors
            # equally. May need to go back to the heading correction step (later TODO)
            if self.calibration_max_accel > 0.05: # TODO is this good
                self.LEFT_MOT_STOP -= 0.01
                self.RIGHT_MOT_STOP -= 0.01
            elif self.calibration_max_accel < -0.05: # TODO is this good
                self.LEFT_MOT_STOP += 0.01
                self.RIGHT_MOT_STOP += 0.01

        # once done have the zero values Calibrated
        self.get_logger().info(f'Zeroes calibrated, offsets: Right: {self.RIGHT_MOT_STOP}, Left: {self.LEFT_MOT_STOP}')

        # Now calibrate max fwd by setting both to max fwd for 2 sec measuring heading difference
        # keep reducing the one side until it goes straight.
        self.get_logger().info('Calibrating forward magnitude')
        ang_error = 100
        self.LEFT_MOT_FWD_MAX = 1.0
        self.RIGHT_MOT_FWD_MAX = 1.0
        while abs(ang_error) > 3:
            # Stop bot and get starting angle
            self.drive_enable_publisher.publish(disable_msg) # Stop the bot for a bit to get the heading
            time.sleep(0.5)
            start_ang = self.cur_heading
            self.drive_enable_publisher.publish(enable_msg)
            drive_msg.left_throttle = self.LEFT_MOT_FWD_MAX
            drive_msg.right_throttle = self.RIGHT_MOT_FWD_MAX
            self.drive_cmd_publisher.publish(drive_msg)
            # Wait for 3 seconds then see which way it is turning.
            time.sleep(3)
            self.drive_enable_publisher.publish(disable_msg) # Stop the bot
            ang_error = start_ang - self.cur_heading
            # Correct in case angle went from a small positive to ~ 360 or opposite
            if ang_error > 180:
                ang_error -= 360
            elif ang_error < -180:
                ang_error += 360
            # increase and decrease the wheel values until heading is not changing
            if ang_error > 3.0:
                if self.LEFT_MOT_FWD_MAX == 1.0 and self.RIGHT_MOT_FWD_MAX != 1.0:
                    # odd case where more finely back increase right to keep one at 1.0
                    self.RIGHT_MOT_FWD_MAX += 0.01
                    # Prevents both going to zero or something weird
                else:
                    self.LEFT_MOT_FWD_MAX -= 0.02
                
            elif ang_error < -3.0:
                if self.RIGHT_MOT_FWD_MAX == 1.0 and self.LEFT_MOT_FWD_MAX != 1.0:
                    # odd case where more finely back increase right to keep one at 1.0
                    self.LEFT_MOT_FWD_MAX += 0.01
                    # Prevents both going to zero or something weird
                else:
                    self.RIGHT_MOT_FWD_MAX -= 0.02
            
            # Drive backwards a bit to not run out of space, doesn't have to be a line
            self.drive_enable_publisher.publish(enable_msg)
            drive_msg.left_throttle = self.LEFT_MOT_BACK_MAX
            drive_msg.right_throttle = self.RIGHT_MOT_BACK_MAX
            self.drive_cmd_publisher.publish(drive_msg)
            time.sleep(3)
            self.drive_enable_publisher.publish(disable_msg) # Stop the bot

        self.get_logger().info(f'Max Fwd calibrated: Right: {self.RIGHT_MOT_FWD_MAX}, Left: {self.LEFT_MOT_FWD_MAX}')

        # Finally calibrate the same for going backwards
        self.get_logger().info('Calibrating Backwards magnitude')
        ang_error = 100
        self.LEFT_MOT_BACK_MAX = -1.0
        self.RIGHT_MOT_BACK_MAX = -1.0
        while abs(ang_error) > 3:
            # Stop bot and get starting angle
            self.drive_enable_publisher.publish(disable_msg) # Stop the bot for a bit to get the heading
            time.sleep(0.5)
            start_ang = self.cur_heading
            self.drive_enable_publisher.publish(enable_msg)
            drive_msg.left_throttle = self.LEFT_MOT_BACK_MAX
            drive_msg.right_throttle = self.RIGHT_MOT_BACK_MAX
            self.drive_cmd_publisher.publish(drive_msg)
            # Wait for 3 seconds then see which way it is turning.
            time.sleep(3)
            self.drive_enable_publisher.publish(disable_msg) # Stop the bot
            ang_error = start_ang - self.cur_heading
            # Correct in case angle went from a small positive to ~ 360 or opposite
            if ang_error > 180:
                ang_error -= 360
            elif ang_error < -180:
                ang_error += 360
            # increase and decrease the wheel values until heading is not changing
            if ang_error < -3.0:
                if self.LEFT_MOT_BACK_MAX == 1.0 and self.RIGHT_MOT_BACK_MAX != 1.0:
                    # odd case where more finely back increase right to keep one at -1.0
                    self.RIGHT_MOT_BACK_MAX -= 0.01
                    # Prevents both going to zero or something weird
                else:
                    self.LEFT_MOT_BACK_MAX += 0.02
                
            elif ang_error > 3.0:
                if self.RIGHT_MOT_BACK_MAX == 1.0 and self.LEFT_MOT_BACK_MAX != 1.0:
                    # odd case where more finely back increase right to keep one at -1.0
                    self.LEFT_MOT_BACK_MAX -= 0.01
                    # Prevents both going to zero or something weird
                else:
                    self.RIGHT_MOT_BACK_MAX += 0.02
            
            # Drive forewards a bit to not run out of space, doesn't have to be a line (but should be now)
            self.drive_enable_publisher.publish(enable_msg)
            drive_msg.left_throttle = self.LEFT_MOT_FWD_MAX
            drive_msg.right_throttle = self.RIGHT_MOT_FWD_MAX
            self.drive_cmd_publisher.publish(drive_msg)
            time.sleep(3)
            self.drive_enable_publisher.publish(disable_msg) # Stop the bot

        self.get_logger().info(f'Max Back calibrated: Right: {self.RIGHT_MOT_BACK_MAX}, Left: {self.LEFT_MOT_BACK_MAX}')

        # Save calibrations to a motor_params.yaml file to be stored for future launches
        motor_calibration_file = open('/home/oscar1/git/IMOfficeBot/launch/motor_params.yaml', 'w')
        motor_calibration_file.write('/controller_node:\n  ros__parameters:\n    '
                                     + f'LEFT_MOT_STOP: {self.LEFT_MOT_STOP}\n    '
                                     + f'LEFT_MOT_FWD_MAX: {self.LEFT_MOT_FWD_MAX}\n    '
                                     + f'LEFT_MOT_BACK_MAX: {self.LEFT_MOT_BACK_MAX}\n    '
                                     + f'RIGHT_MOT_STOP: {self.RIGHT_MOT_STOP}\n    '
                                     + f'RIGHT_MOT_FWD_MAX: {self.RIGHT_MOT_FWD_MAX}\n    '
                                     + f'RIGHT_MOT_BACK_MAX: {self.RIGHT_MOT_BACK_MAX}\n')
        motor_calibration_file.close()

        self.mode = ControllerMode.IDLE
        self.loop_timer.reset()


    def teleop_loop(self):

        # TODO Use the max fwd/back and zero calibrated values
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

        # map the throttle to between the zero and max fwd/back
        # Controller throttle is between -1 and 1, so can just multiply
        # just in case saturate to this
        if left_throttle > 1.0:
            left_throttle = 1.0
        elif left_throttle < -1.0:
            left_throttle = -1.0
        
        if right_throttle > 1.0:
            right_throttle = 1.0
        elif right_throttle < -1.0:
            right_throttle = -1.0
        
        # Scale outputs using calibration values
        if left_throttle > 0.0:
            # Find magnitude of positive range, scale throttle, and add offset
            left_pos_magnitude = self.LEFT_MOT_FWD_MAX - self.LEFT_MOT_STOP
            left_magnitude = left_pos_magnitude * left_throttle
            left_throttle = left_magnitude + self.LEFT_MOT_STOP
        else :
            # Find magnitude of negative range, scale throttle, and add offset
            left_neg_magnitude = -1* (self.LEFT_MOT_BACK_MAX - self.LEFT_MOT_STOP) # want positive sign here
            left_magnitude = left_neg_magnitude * left_throttle # left throttle is - so sign = -
            left_throttle = left_magnitude + self.LEFT_MOT_STOP
        

        if right_throttle > 0.0:
            # Find magnitude of positive range, scale throttle, and add offset
            right_pos_magnitude = self.RIGHT_MOT_FWD_MAX - self.RIGHT_MOT_STOP
            right_magnitude = right_pos_magnitude * right_throttle
            right_throttle = right_magnitude + self.RIGHT_MOT_STOP
        else :
            # Find magnitude of negative range, scale throttle, and add offset
            right_neg_magnitude = -1* (self.RIGHT_MOT_BACK_MAX - self.RIGHT_MOT_STOP) # want positive sign here
            right_magnitude = right_neg_magnitude * right_throttle # right throttle is - so sign = -
            right_throttle = right_magnitude + self.RIGHT_MOT_STOP

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
