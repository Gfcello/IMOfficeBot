import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
       '.',
       'launch',
       'params.yaml'
    )
    motor_config = os.path.join(
        '.',
        'launch',
        'motor_params.yaml'
    )

    return LaunchDescription([
      Node(
         package='controller',
         executable='controller_node',
         name='controller_node',
         parameters=[config, motor_config]
      ),
      # Node(
      #    package='core',
      #    executable='mode_node',
      #    name='mode_node',
      #    parameters=[config]
      # ),
      # Node(
      #    package='diagnostics',
      #    executable='diagnostic_node',
      #    name='diagnostic_node',
      #    parameters=[config]
      # ),
      # Node(
      #    package='diagnostics',
      #    executable='recorder_node',
      #    name='recorder_node',
      #    parameters=[config]
      # ),
      Node(
         package='sensors',
         executable='imu_node',
         name='imu_node',
         parameters=[config]
      ),
      Node(
         package='outputs',
         executable='driving_node',
         name='driving_node',
         parameters=[config]
      ),
   ])
