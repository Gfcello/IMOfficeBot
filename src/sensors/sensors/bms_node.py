import rclpy
from rclpy.node import Node
import lgpio
import time

from message_types.msg import NodeStatus
from std_msgs.msg import Float64


class BMSNode(Node):
    # Node communicates with ADS 1015 ADC over I2C to read voltage and current

    # Device constants:
    I2C_ADDR = 0x48
    CONVERSION_REGISTER = 0x00
    CONFIG_REGISTER = 0x01

    DATA_LEN = 12
    MAX_VOLTAGE = 4.096

    SINGLE_READ_CMD = 0b1
    A0_CHANNEL = 0b100
    A1_CHANNEL = 0b101
    A2_CHANNEL = 0b110
    A3_CHANNEL = 0b111
    GAIN_AMP = 0b001
    OP_MODE = 0b1  # single shot
    DATA_RATE = 0b011
    COMP_MODE = 0b0
    COMP_POL = 0b0
    COMP_LAT = 0b0
    COMP_QUE = 0b00  # 1 conversion

    CURRENT_CHANNEL = A0_CHANNEL
    VOLTAGE_CHANNEL = A1_CHANNEL

    def __init__(self):
        super().__init__('bms_node')
        self.voltage_publisher = self.create_publisher(Float64, 'bms_node/voltage', 10)
        self.current_publisher = self.create_publisher(Float64, 'bms_node/current', 10)
        self.status_publisher = self.create_publisher(NodeStatus, 'bms_node/status', 10)
        self.status_msg = NodeStatus()

        self.i2c_handle = lgpio.i2c_open(1, self.I2C_ADDR)
        self.get_logger().debug('Power Sense ADC handle opened')

        timer_period = 1.0  # seconds
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Read the newest sensor data
        [voltage_reading, current_reading] = self.read_adc()

        # self.get_logger().debug(f'Raw Data: {data_buffer}')

        self.status_msg.status = NodeStatus.STATUS_GOOD
        if current_reading > 60.0:
            self.get_logger().error('Dangrously High Current Draw')
            self.status_msg.status = NodeStatus.STATUS_APOCALYPSE
        elif current_reading > 50.0:
            self.get_logger().warn('High Current Draw')
            self.status_msg.status = max(self.status_msg.status, NodeStatus.STATUS_WARN)

        if voltage_reading < 13.1:
            self.get_logger().error('Battery Dangerously Low, Turn off Thrusters!')
            self.status_msg.status = NodeStatus.STATUS_APOCALYPSE
        elif voltage_reading < 14.0:
            self.get_logger().warn('Battery Running Low')
            self.status_msg.status = max(self.status_msg.status, NodeStatus.STATUS_WARN)

        # Publish the most recent status
        self.status_publisher.publish(self.status_msg)
        # Publish voltage and current
        v_msg = Float64()
        v_msg.data = voltage_reading
        self.voltage_publisher.publish(v_msg)
        c_msg = Float64()
        c_msg.data = current_reading
        self.current_publisher.publish(c_msg)

    def configure_sensor(self, channel):
        config_cmd = ((self.SINGLE_READ_CMD << 15) | (channel << 12) | (self.GAIN_AMP << 9) |
                      (self.OP_MODE << 8) | (self.DATA_RATE << 5) | (self.COMP_MODE << 4) |
                      (self.COMP_POL << 3) | (self.COMP_LAT << 2) | (self.COMP_QUE))
        self.write_register(self.CONFIG_REGISTER, config_cmd)

    def to_voltage(self, value):
        volts = self.MAX_VOLTAGE * value
        volts /= ((2 ** (self.DATA_LEN - 1)) - 1)
        # Conversion from https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
        blue_robotics_voltage = 11.0 * volts
        return blue_robotics_voltage

    def to_current(self, value):
        volts = self.MAX_VOLTAGE * value
        volts /= ((2 ** (self.DATA_LEN - 1)) - 1)
        # Conversion from https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
        blue_robotics_current = (37.8788 * volts) + 0.33
        return blue_robotics_current

    def read_adc(self):
        # First set the register to read
        self.configure_sensor(self.VOLTAGE_CHANNEL)
        time.sleep(0.01)  # delay 10ms for reading
        voltage_buffer = self.read_register(self.CONVERSION_REGISTER)
        voltage = int.from_bytes(voltage_buffer, "big") >> (16 - self.DATA_LEN)
        if voltage >= (2 ** (self.DATA_LEN - 1)):
            voltage = voltage - (2 ** self.DATA_LEN)
        bat_voltage = self.to_voltage(voltage)

        time.sleep(0.01)
        self.configure_sensor(self.CURRENT_CHANNEL)
        time.sleep(0.01)  # delay 10ms for reading
        current_buffer = self.read_register(self.CONVERSION_REGISTER)
        current = int.from_bytes(current_buffer, "big") >> (16 - self.DATA_LEN)
        if current >= (2 ** (self.DATA_LEN - 1)):
            current = current - (2 ** self.DATA_LEN)
        bat_current = self.to_current(current)

        return [bat_voltage, bat_current]

    def write_register(self, register, value):
        command = [(value >> 8) & 0xFF, value & 0xFF]
        try:
            self.get_logger().debug(f'ADC Writing: {command}')
            lgpio.i2c_write_i2c_block_data(self.i2c_handle, register, command)
        except lgpio.error as e:
            self.get_logger().error(f'Power Sense ADC Sensor write failure: {e}')
            self.status_msg.status = NodeStatus.STATUS_ERROR

    def read_register(self, register):
        try:
            data = lgpio.i2c_read_i2c_block_data(self.i2c_handle, register, 2)[1]
            return data
        except lgpio.error as e:
            self.get_logger().error(f'Power Sense ADC read failure: {e}')
            self.status_msg.status = NodeStatus.STATUS_ERROR
            return None


def main(args=None):
    rclpy.init(args=args)

    bms_node = BMSNode()

    rclpy.spin(bms_node)


if __name__ == '__main__':
    main()
