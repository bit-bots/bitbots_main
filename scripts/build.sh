#!/bin/bash

if [[ -d "$HOME/catkin_ws" ]]; then
    ROS_WORKSPACE="$HOME/catkin_ws"
    echo "ROS Workspace found at:    ${ROS_WORKSPACE}"
else
    read -p "Where is your catkin workspace?  " ROS_WORKSPACE
fi


cd ${ROS_WORKSPACE}
source devel/setup.bash
catkin build

