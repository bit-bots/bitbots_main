#!/bin/bash

while true ; do
    (
        . /home/odroid/boot-defaults.sh

        if $START_BEHAVIOUR ; then
            roslaunch bitbots_body_behaviour start_behaviour_standalone.launch
        fi
    )
    echo "You have two seconds to kill this script."
    sleep 2 || exit $?
done

