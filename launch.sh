#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $DIR

tmux new-session -s mrdc   -n "roscore" -d './launch_scripts/launch_roscore.sh'
tmux new-window  -t mrdc:1 -n "ros_serial" './launch_scripts/launch_ros_serial.sh'
sleep 5
./launch_scripts/load_rosparam.sh
tmux new-window  -t mrdc:2 -n "xbee" './launch_scripts/launch_xbee.sh'
tmux new-window  -t mrdc:3 -n "controls" './launch_scripts/launch_controls.sh'
tmux new-window  -t mrdc:4 -n "pca6985" './launch_scripts/launch_pca9685.sh'
tmux attach-session -t mrdc
