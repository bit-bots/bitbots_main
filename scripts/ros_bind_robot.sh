#!/bin/bash

# This script sets the ROS_DOMAIN_ID environment variable to the value associated with the provided robot name.
# It also sets environment variables to enable automatic discovery of nodes in the local network.

# Usage: source robot_connect.sh <robot_name|ROS_DOMAIN_ID>

# Detect if this script is being sourced or run as a command
if [ "$0" = "$BASH_SOURCE" ]; then
    echo "This script must be sourced, not executed. Use: source $0 <robot_name|ROS_DOMAIN_ID>"
    exit 1
fi

# Define a map of robot names to ROS_DOMAIN_IDs
declare -A robot_map
robot_map[amy]=11
robot_map[rory]=12
robot_map[jack]=13
robot_map[donna]=14
robot_map[melody]=15
robot_map[rose]=16

# Check if the provided argument is a valid robot name or a numeric ROS_DOMAIN_ID
if ! ( [ -n "${robot_map[$1]}" ] || [[ $1 =~ ^[0-9]+$ ]]); then
    echo "Usage: source $0 <robot_name|ROS_DOMAIN_ID>"
    return 1
fi

echo "Setting up ROS connection..."

# If the provided argument is a valid robot name, set the ROS_DOMAIN_ID
if [ -n "${robot_map[$1]}" ]; then
    export ROS_DOMAIN_ID=${robot_map[$1]}
    echo "Binding to robot '$1' with ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
else
    export ROS_DOMAIN_ID=$1
    echo "Binding to robot with ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

# Set discovery range to local network
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Unset local variables that may interfere with the discovery process
unset ROS_LOCALHOST_ONLY

echo "Restarting ros2 daemon..."
ros2 daemon stop

echo "Configuration complete!"

# Print a list of discovered nodes if there are any
if [ -n "$(ros2 node list 2>/dev/null)" ]; then
    echo "\nDiscovered nodes:"
    ros2 node list
fi
