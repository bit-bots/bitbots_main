#!/bin/bash

# 1 param is the remote device
# 2 param is the ros master
# 3 param is the ros_package
# 4 param is the launch file
# Following params are the roslaunch arguments

ssh -t -t $0 'export ROS_MASTER_URI=http://$1:11311/; zsh -o HUP -c "roslaunch ${@:2}"'
