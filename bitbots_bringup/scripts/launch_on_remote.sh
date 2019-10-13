#!/bin/bash

##########
# Params #
##########
# 1 param is the remote device
# 2 param is the ros master
# 3 param is the ros_package
# 4 param is the launch file
# Following params are the roslaunch arguments

ssh -t -t $1 'source ~/.zshrc; export ROS_MASTER_URI=http://'$2':11311/; zsh -o HUP -c "roslaunch '${*:3}'"'
