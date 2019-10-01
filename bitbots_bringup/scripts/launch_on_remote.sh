#!/bin/bash

ssh -t -t jetson 'export ROS_MASTER_URI=http://nuc:11311/; zsh -o HUP -c "roslaunch bitbots_vision vision_startup.launch debug:=true"'
