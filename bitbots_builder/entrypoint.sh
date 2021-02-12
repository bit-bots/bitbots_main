#!/bin/bash
set -e
set -x

source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash

exec "$@"