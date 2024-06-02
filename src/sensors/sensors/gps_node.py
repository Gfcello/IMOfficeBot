import rclpy
from rclpy.node import Node

import io
import pynmea2
import serial

from message_types.msg import GpsCoords, NodeStatus


class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.declare_parameter('mount_point', '/dev/ttyACM0')
        self.mount_point = self.get_parameter('mount_point').get_parameter_value().string_value

        self.coordinate_publisher = self.create_publisher(GpsCoords, 'gps_node/gps_coords', 10)
        self.radio_data_publisher = self.create_publisher(GpsCoords,
                                                          'radio_node/send/gps_data', 10)
        self.coord_msg = GpsCoords()

        self.status_publisher = self.create_publisher(NodeStatus, 'gps_node/status', 10)
        self.status_msg = NodeStatus()

        self.ser = serial.Serial(self.mount_point, 9600, timeout=5.0)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))

        self.last_timestamp = 0
        self.spins_since_last_coords = 0

        timer_period = 0.5  # seconds
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        coord_msg_found = False

        while self.ser.in_waiting > 0:  # While there are bytes in the input buffer
            try:
                line = self.sio.readline()
                gps_data = pynmea2.parse(line)
            except serial.SerialException as e:
                self.get_logger().error('GPS Device error: {}'.format(e))
            except pynmea2.ParseError as e:
                self.get_logger().error('GPS Parse error: {}'.format(e))

            # if lat/long data is contained in the buffer is more recent overwrite it
            if hasattr(gps_data, 'latitude') and hasattr(gps_data, 'longitude') \
                    and (gps_data.timestamp != self.last_timestamp):
                coord_msg_found = True
                self.coord_msg.latitude = gps_data.latitude
                self.coord_msg.longitude = gps_data.longitude

                self.get_logger().debug(f'Publishing msg recieved at: {gps_data.timestamp}')

                self.coordinate_publisher.publish(self.coord_msg)
                # Also publish gps data to radio to send to mothership
                self.radio_data_publisher.publish(self.coord_msg)

                self.status_msg.status = self.status_msg.STATUS_GOOD
                self.status_publisher.publish(self.status_msg)
                self.last_timestamp = gps_data.timestamp
                self.spins_since_last_coords = 0

        # Have now read through each of the recieved messages
        if not coord_msg_found:
            # Check if number of spins without coords exceeds 3
            self.spins_since_last_coords += 1
            if self.spins_since_last_coords >= 5:
                self.get_logger().warn(f'No Lat/Long in {self.spins_since_last_coords / 2} s')
                # Check what status based on number of missed messages
                if self.spins_since_last_coords >= 10:
                    self.status_msg.status = self.status_msg.STATUS_ERROR
                    self.status_publisher.publish(self.status_msg)
                else:
                    self.status_msg.status = self.status_msg.STATUS_WARN
                    self.status_publisher.publish(self.status_msg)


def main(args=None):
    rclpy.init(args=args)

    gps_node = GpsNode()

    rclpy.spin(gps_node)


if __name__ == '__main__':
    main()
