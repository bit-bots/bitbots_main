#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash

exec "$@"