#! /bin/bash
[[ "${BASH_SOURCE[0]}" == "${0}" ]] && echo -e "This script can be used to connect directly to the robot.\nIt sets the \$ROS_IP variable. See http://wiki.ros.org/ROS/NetworkSetup for more information.\n\n\033[1m\033[91mUse \"source ${BASH_SOURCE[0]}\" instead of executing the script directly.\033[0m" && exit 1
export ROS_IP=`ip a | grep 'inet 192\.168\.[123]\.' | awk '{print $2}' | cut -d '/' -f 1`
echo "Set \$ROS_IP to ${ROS_IP}"
echo "To deactivate use \"unset ROS_IP\""
