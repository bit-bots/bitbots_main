#! /bin/bash
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
   echo "This script can be used to connect directly to the robot."
   echo "It sets the \$ROS_IP variable. See http://wiki.ros.org/ROS/NetworkSetup for more information."
   echo
   echo -e "\033[1m\033[91mUse \"source ${BASH_SOURCE[0]}\" instead of executing the script directly.\033[0m"
   exit 1
fi

_IP=$(ip a | grep 'inet 192\.168\.[0123]\.' | awk '{print $2}' | cut -d '/' -f 1)

if [[ -z $_IP ]]; then
    echo "No network available"
    unset _IP
else
    export ROS_IP=$_IP
    echo "Set \$ROS_IP to ${ROS_IP}"
    echo "To deactivate use \"unset ROS_IP\""
fi
