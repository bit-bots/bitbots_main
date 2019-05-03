#!/bin/bash

#
# This script gets executed on boot if the systemd service 'start_behavior.service' is enabled
#

if hash roslaunch 2>/dev/null; then
    roslaunch bitbots_bringup teamplayer.launch
else
    echo "Please source the workspace"
    exit 1
fi
