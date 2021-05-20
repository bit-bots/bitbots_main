#!/bin/bash

source $(dirname "$0")/setenvs.sh
exec $(dirname "$0")/start_webots_ros_supervisor.py "$@"
