#!/bin/bash

#
# This script gets executed on boot if the systemd service 'start_roscore.service' is enabled
#

if hash roslaunch 2>/dev/null; then
    screen -S motion roslaunch bitbots_bringup motion.launch
else
    echo "Please source the workspace"
    exit 1
fi
