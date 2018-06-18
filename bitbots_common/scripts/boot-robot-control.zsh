#!/bin/zsh

source ~/boot-configuration.sh
source $WORKSPACE/devel/setup.zsh

while true ; do
    (
        . ~/boot-configuration.sh
        if $START_MOTION; then
            if [[ $ROBOT != "wolfgang" || $HOST == nuc* ]]; then
                roslaunch bitbots_common motion.launch $ROBOT:=true
            fi
        fi
    )
    echo "You have two seconds to kill this script."
    sleep 2 || exit $?
done
