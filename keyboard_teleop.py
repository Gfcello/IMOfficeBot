#!/usr/bin/env python3

import rclpy

from std_msgs.msg import Bool
from message_types.msg import TeleopCmd

import keyboard
import time

motors_enabled = True
fwd_throttle = 0.0
cw_vel = 0.0


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('gps_node')

    teleop_publisher = node.create_publisher(TeleopCmd, 'teleop_cmd', 10)
    enable_publisher = node.create_publisher(Bool, 'mode_node/enable_motors', 10)

    print('wasd controls, q to stop and disable driving, space to toggle enabling driving and stop')
    
    keyboard.add_hotkey('w', lambda: on_key_callback('w', teleop_publisher, enable_publisher))
    keyboard.add_hotkey('a', lambda: on_key_callback('a', teleop_publisher, enable_publisher))
    keyboard.add_hotkey('s', lambda: on_key_callback('s', teleop_publisher, enable_publisher))
    keyboard.add_hotkey('d', lambda: on_key_callback('d', teleop_publisher, enable_publisher))
    keyboard.add_hotkey('space', lambda: on_key_callback('space', teleop_publisher, enable_publisher))
    keyboard.add_hotkey('q', lambda: on_key_callback('q', teleop_publisher, enable_publisher))

    keyboard.add_hotkey('c', lambda: on_key_callback('c', teleop_publisher, enable_publisher)) # calibrate, maybe a different script

    keyboard.wait()

def on_key_callback(key, teleop_publisher, enable_publisher):
    global motors_enabled, fwd_throttle, cw_vel
    match key:
        case 'w':
            fwd_throttle += 0.05
        case 'a':
            cw_vel -= 5.0 # deg / s
        case 's':
            fwd_throttle -= 0.05
        case 'd':
            cw_vel += 5.0 # deg / s
        case 'space':
            fwd_throttle = 0.0
            cw_vel = 0.0
            motors_enabled = not motors_enabled
            msg = Bool()
            msg.data = motors_enabled
            enable_publisher.publish(msg)
        case 'q':
            fwd_throttle = 0.0
            cw_vel = 0.0
            motors_enabled = False
            msg = Bool()
            msg.data = motors_enabled
            enable_publisher.publish(msg)
        case __:
            # If somehow another key is entered don't send a new cmd
            return
    
    msg = TeleopCmd()
    msg.fwd_throttle = fwd_throttle
    msg.cw_deg_per_s = cw_vel
    print(f'fwd: {fwd_throttle}, cw: {cw_vel}, enabled: {motors_enabled}')


if __name__ == '__main__':
    main()