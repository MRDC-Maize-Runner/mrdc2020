#!/bin/bash
cd ros
source devel/setup.sh
cd src/ros_serial #for config.yml
rosrun ros_serial ros_serial_node
/bin/bash
