import rclpy
from rclpy.node import Node
from datetime import datetime

from message_types.msg import CoreMode, GpsCoords, ImuData, SystemStatus
from std_msgs.msg import Float64


class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')

        self.declare_parameter('record_topics_str', ['diagnostic_node/system_status'])
        # Read topics to record as a string separated by commas from the config
        # default to just system status array
        self.topics_list = self.get_parameter('record_topics_str').value

        # Prefix log files with time logging node starts
        nanos = self.get_clock().now().nanoseconds
        time_obj = datetime.fromtimestamp(nanos // 1000000000)
        self.file_prefix = time_obj.strftime('%Y-%m-%d_%H-%M-%S')

        # Create empty array of open file list
        self.files_list = [0] * len(self.topics_list)
        self.get_logger().info(f'{self.files_list}')

        # Unfortunately don't have a way to get message topic types, so have to hard code:

        # Subscribers for important topics to log are here if they are requested
        if 'diagnostic_node/system_status' in self.topics_list:
            topic_idx_1 = self.topics_list.index('diagnostic_node/system_status')
            self.create_subscription(SystemStatus, 'diagnostic_node/system_status',
                                     lambda msg: self.msg_callback(msg, topic_idx_1), 10)
            file_name = self.file_prefix + '_diagnostic_node_system_status.csv'
            self.files_list[topic_idx_1] = open(file_name, 'w')

        if 'thruster_1_node/throttle' in self.topics_list:
            topic_idx_2 = self.topics_list.index('thruster_1_node/throttle')
            self.create_subscription(Float64, 'thruster_1_node/throttle',
                                     lambda msg: self.msg_callback(msg, topic_idx_2), 10)
            file_name = self.file_prefix + '_thruster_1_node_throttle.csv'
            self.files_list[topic_idx_2] = open(file_name, 'w')

        if 'thruster_2_node/throttle' in self.topics_list:
            topic_idx_3 = self.topics_list.index('thruster_2_node/throttle')
            self.create_subscription(Float64, 'thruster_2_node/throttle',
                                     lambda msg: self.msg_callback(msg, topic_idx_3), 10)
            file_name = self.file_prefix + '_thruster_2_node_throttle.csv'
            self.files_list[topic_idx_3] = open(file_name, 'w')

        if 'gps_node/gps_coords' in self.topics_list:
            topic_idx_4 = self.topics_list.index('gps_node/gps_coords')
            self.create_subscription(GpsCoords, 'gps_node/gps_coords',
                                     lambda msg: self.msg_callback(msg, topic_idx_4), 10)
            file_name = self.file_prefix + '_gps_node_gps_coords.csv'
            self.files_list[topic_idx_4] = open(file_name, 'w')

        if 'mode_node/mode' in self.topics_list:
            topic_idx_5 = self.topics_list.index('mode_node/mode')
            self.create_subscription(CoreMode, 'mode_node/mode',
                                     lambda msg: self.msg_callback(msg, topic_idx_5), 10)
            file_name = self.file_prefix + '_mode_node_mode.csv'
            self.files_list[topic_idx_5] = open(file_name, 'w')

        if 'mode_node/gps_target' in self.topics_list:
            topic_idx_6 = self.topics_list.index('mode_node/gps_target')
            self.create_subscription(GpsCoords, 'mode_node/gps_target',
                                     lambda msg: self.msg_callback(msg, topic_idx_6), 10)
            file_name = self.file_prefix + '_mode_node_gps_target.csv'
            self.files_list[topic_idx_6] = open(file_name, 'w')

        if 'bms_node/voltage' in self.topics_list:
            topic_idx_7 = self.topics_list.index('bms_node/voltage')
            self.create_subscription(Float64, 'bms_node/voltage',
                                     lambda msg: self.msg_callback(msg, topic_idx_7), 10)
            file_name = self.file_prefix + '_bms_node_voltage.csv'
            self.files_list[topic_idx_7] = open(file_name, 'w')

        if 'bms_node/current' in self.topics_list:
            topic_idx_8 = self.topics_list.index('bms_node/current')
            self.create_subscription(Float64, 'bms_node/current',
                                     lambda msg: self.msg_callback(msg, topic_idx_8), 10)
            file_name = self.file_prefix + '_bms_node_current.csv'
            self.files_list[topic_idx_8] = open(file_name, 'w')

        if 'humidity_node/temp' in self.topics_list:
            topic_idx_9 = self.topics_list.index('humidity_node/temp')
            self.create_subscription(Float64, 'humidity_node/temp',
                                     lambda msg: self.msg_callback(msg, topic_idx_9), 10)
            file_name = self.file_prefix + '_humidity_node_temp.csv'
            self.files_list[topic_idx_9] = open(file_name, 'w')
        
        if 'imu_node/data' in self.topics_list:
            topic_idx_10 = self.topics_list.index('imu_node/data')
            self.create_subscription(ImuData, 'imu_node/data',
                                     lambda msg: self.msg_callback(msg, topic_idx_10), 10)
            file_name = self.file_prefix + '_imu_node_data.csv'
            self.files_list[topic_idx_10] = open(file_name, 'w')

    def msg_callback(self, msg, topic_idx):
        # Write to file the time and the message data
        file_row = str(self.get_clock().now().nanoseconds) + ',' + str(msg) + '\n'
        self.files_list[topic_idx].write(file_row)

    def close_files(self):
        # A function to be called on shutdown to close all files
        for file in self.files_list:
            file.close()


def main(args=None):
    rclpy.init(args=args)

    recorder_node = RecorderNode()
    rclpy.get_default_context().on_shutdown(recorder_node.close_files)

    rclpy.spin(recorder_node)


if __name__ == '__main__':
    main()
