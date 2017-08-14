#!/bin/bash

source ~/catkin_ws/devel/setup.sh

while true ; do
    (
        . ~/catkin_ws/src/bitbots_meta/bitbots_misc/bitbots_common/scripts/boot-defaults.sh

        if $START_BEHAVIOUR ; then
            roslaunch bitbots_common start_robocup_teamplayer.launch hcm:=false duty:=TeamPlayer
        fi
    )
    echo "You have two seconds to kill this script."
    sleep 2 || exit $?
done

