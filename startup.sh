#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/debian/ros2_ws/install/setup.bash

# START ROS SERVER
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# START APP
ros2 run autonomy_si listener 

