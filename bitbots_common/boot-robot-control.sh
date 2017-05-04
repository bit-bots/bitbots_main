#!/bin/bash

while true ; do
    (
        roslaunch bitbots_hcm start_hcm_minibot_robot.launch
    )
    echo "You have two seconds to kill this script."
    sleep 2 || exit $?
done

