#! /bin/bash
[[ "${BASH_SOURCE[0]}" == "${0}" ]] && echo "Please use \"source ${BASH_SOURCE[0]}\"" && exit 1
export ROS_IP=`ip a | grep -F 'inet 192.168.1.' | awk '{print $2}' | cut -d '/' -f 1`
echo "Set \$ROS_IP to ${ROS_IP}"
echo "To deactivate use \"unset ROS_IP\""
