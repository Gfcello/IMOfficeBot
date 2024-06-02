import rclpy
from rclpy.node import Node

from message_types.msg import CoreMode, NodeStatus, SystemStatus, TelemetryData
from std_msgs.msg import Float64

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('diagnostic_node')

        self.status_publisher = self.create_publisher(SystemStatus,
                                                      'diagnostic_node/system_status', 10)

        self.telemetry_publisher = self.create_publisher(TelemetryData,
                                                      'radio_node/send/telemetry', 10)

        # A subscriber for each other node in the system is added here
        self.create_subscription(NodeStatus, 'mode_node/status',
                                 lambda msg: self.node_callback(msg, SystemStatus.MODE_NODE), 10)
        self.create_subscription(NodeStatus, 'controller_node/status',
                                 lambda msg: self.node_callback(msg,
                                                                SystemStatus.CONTROLLER_NODE), 10)
        self.create_subscription(NodeStatus, 'thruster_1_node/status',
                                 lambda msg: self.node_callback(msg,
                                                                SystemStatus.THRUSTER_1_NODE), 10)
        self.create_subscription(NodeStatus, 'thruster_2_node/status',
                                 lambda msg: self.node_callback(msg,
                                                                SystemStatus.THRUSTER_2_NODE), 10)
        self.create_subscription(NodeStatus, 'lights_node/status',
                                 lambda msg: self.node_callback(msg,
                                                                SystemStatus.LIGHTS_NODE), 10)
        self.create_subscription(NodeStatus, 'imu_node/status',
                                 lambda msg: self.node_callback(msg, SystemStatus.IMU_NODE), 10)
        self.create_subscription(NodeStatus, 'gps_node/status',
                                 lambda msg: self.node_callback(msg, SystemStatus.GPS_NODE), 10)
        self.create_subscription(NodeStatus, 'leak_node/status',
                                 lambda msg: self.node_callback(msg, SystemStatus.LEAK_NODE), 10)
        self.create_subscription(NodeStatus, 'radio_node/status',
                                 lambda msg: self.node_callback(msg, SystemStatus.RADIO_NODE), 10)
        self.create_subscription(NodeStatus, 'opt_camera_node/status',
                                 lambda msg: self.node_callback(msg,
                                                                SystemStatus.OPT_CAMERA_NODE), 10)
        self.create_subscription(NodeStatus, 'ir_camera_node/status',
                                 lambda msg: self.node_callback(msg,
                                                                SystemStatus.IR_CAMERA_NODE), 10)
        self.create_subscription(NodeStatus, 'bms_node/status',
                                 lambda msg: self.node_callback(msg, SystemStatus.BMS_NODE), 10)
        self.create_subscription(NodeStatus, 'humidity_node/status',
                                 lambda msg: self.node_callback(msg,
                                                                SystemStatus.HUMIDITY_NODE), 10)

        self.create_subscription(Float64, 'humidity_node/temp', self.temperature_callback, 10)
        self.create_subscription(Float64, 'bms_node/voltage', self.voltage_callback, 10)
        self.create_subscription(CoreMode, 'radio_node/send/change_mode', self.mode_callback, 10)

        self.system_status_msg = SystemStatus()
        self.telemetry_msg = TelemetryData()

        timer_period = 0.5  # seconds
        self.create_timer(timer_period, self.timer_callback)

        # Publish telemetry every 
        self.create_timer(10, self.telemetry_callback)

    def timer_callback(self):
        # Publish the most recent statuses for each node
        # Each node reports a worse status in case of timeout
        self.status_publisher.publish(self.system_status_msg)

    def telemetry_callback(self):
        # Publish the telemetry at low freq
        self.telemetry_publisher.publish(self.telemetry_msg)

    def node_callback(self, msg, node_idx):
        # Update the system status message at the node's index
        self.system_status_msg.node_statuses[node_idx] = msg.status

    def voltage_callback(self, msg):
        self.telemetry_msg.voltage = msg.data

    def temperature_callback(self, msg):
        self.telemetry_msg.temperature = msg.data

    def mode_callback(self, msg):
        self.telemetry_msg.mode = msg.mode


def main(args=None):
    rclpy.init(args=args)

    diagnostic_node = DiagnosticNode()

    rclpy.spin(diagnostic_node)


if __name__ == '__main__':
    main()
