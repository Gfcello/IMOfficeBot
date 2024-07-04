# ROS2 Template
This repo holds ROS2-based software to run a driving robot that maps and drives to locations.

The robot requires the sllidar_ros2 library to run the RPLidar, the slam_toolbox library for
mapping, and the nav2 library for navigation and obstacle avoidance.

Required library links:

https://github.com/Slamtec/sllidar_ros2

https://github.com/SteveMacenski/slam_toolbox

https://github.com/ros-navigation/navigation2

A setup process will be made later

# Packages in this Repo
## Core
This is the package for the modes of the robot.
## Controller
This is the package for all the controller code.
## Diagnostics
This is the package for all diagnostics code.
## Sensors
This is the package for all sensor drivers and nodes.
## Outputs
This is the package for system outputs like actuators, lights, etc.
## Message Types
This package defines custom messages used by the different nodes.

# File System Architecture
Will put a mermaid diagram here once one is made
