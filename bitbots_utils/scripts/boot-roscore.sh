#!/bin/bash

#
# This script gets executed on boot if the systemd service 'start_roscore.service' is enabled
#

if hash roscore 2>/dev/null; then
    roscore
else
    echo "Please source the workspace"
    exit 1
fi
