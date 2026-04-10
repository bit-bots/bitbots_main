#!/bin/env bash

# setup env
source devel/setup.bash

# run with riviz or not
roslaunch yesense_imu yesense_rviz.launch &

# set tf
# usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds)
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 fixed_frame imu_link 100

