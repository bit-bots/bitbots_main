#!/bin/bash

# This script sets the ROS_DOMAIN_ID and ZENOH_CONFIG_OVERRIDE environment variables.
#
# Usage: 
#   source robot_connect.sh <robot_name>
#   source robot_connect.sh <ROS_DOMAIN_ID> <IP_ADDRESS>

# Detect if this script is being sourced or executed
if [ "$0" = "${BASH_SOURCE[0]}" ]; then
    echo "This script must be sourced, not executed. Use: source $0 <robot_name> | <ROS_DOMAIN_ID IP_ADDRESS>"
    exit 1
fi

if [ -z "$1" ]; then
    echo "Usage:"
    echo "  source $0 <robot_name>"
    echo "  source $0 <ROS_DOMAIN_ID> <IP_ADDRESS>"
    return 1
fi

# Define mappings: "IP ROS_DOMAIN_ID"
declare -A robot_map
robot_map[kalliope]="172.20.1.11 11"
robot_map[mickey]="172.20.1.12 12"
robot_map[pink]="172.20.1.13 13"
robot_map[romeo]="172.20.1.14 14"
robot_map[carrie]="172.20.1.15 15"
robot_map[peter]="172.20.1.16 16"
robot_map[kiki]="172.20.1.17 17"
robot_map[martyn]="172.20.1.18 18"
robot_map[comp_kalliope]="10.0.6.1 11"
robot_map[comp_mickey]="10.0.6.2 12"
robot_map[comp_pink]="10.0.6.3 13"
robot_map[comp_romeo]="10.0.6.4 14"
robot_map[comp_carrie]="10.0.6.5 15"
robot_map[comp_peter]="10.0.6.6 16"
robot_map[comp_kiki]="10.0.6.7 17"
robot_map[comp_martyn]="10.0.6.8 18"

INPUT="$1"

if [ -n "${robot_map[$INPUT]}" ]; then
    # Lookup values from the pre-configured map
    read -r ROBOT_IP ROBOT_DOMAIN <<< "${robot_map[$INPUT]}"
    
    echo "Binding to robot '$INPUT' (IP: $ROBOT_IP, ROS_DOMAIN_ID: $ROS_DOMAIN_ID)"

elif [[ "$INPUT" =~ ^[0-9]+$ ]]; then
    # Numeric Domain ID mode - requires explicit IP as argument 2
    ROBOT_DOMAIN="$INPUT"
    ROBOT_IP="$2"
    
    if [ -z "$ROBOT_IP" ]; then
        echo "Error: When providing a numeric ROS_DOMAIN_ID directly, you must also pass the IP address."
        echo "Usage: source $0 $INPUT <IP_ADDRESS>"
        return 1
    fi
    
    echo "Binding directly to ROS_DOMAIN_ID=$ROS_DOMAIN_ID (IP: $ROBOT_IP)"

else
    echo "Error: '$INPUT' is not a valid robot name or numeric ROS_DOMAIN_ID."
    return 1
fi

export ROS_DOMAIN_ID="$ROBOT_DOMAIN"
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${ROBOT_IP}:7447\"]"

echo "Restarting ros2 daemon..."
ros2 daemon stop

echo "Configuration complete!"

# Print discovered nodes if active
if [ -n "$(ros2 node list 2>/dev/null)" ]; then
    echo -e "\nDiscovered nodes:"
    ros2 node list
fi
