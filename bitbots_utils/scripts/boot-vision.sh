#!/bin/bash

#
# This script gets executed on boot if the systemd service 'start_vision.service' is enabled
#

if hash roslaunch 2>/dev/null; then
    roslaunch bitbots_vision vision_startup.launch
else
    echo "Please source the workspace"
    exit 1
fi
