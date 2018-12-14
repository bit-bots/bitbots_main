#!/bin/zsh

source ~/boot-configuration.sh
source $WORKSPACE/devel/setup.zsh

while true ; do
    (
        . ~/boot-configuration.sh
        if $START_BEHAVIOUR ; then
            if [[ $ROBOT != "wolfgang" ]]; then
                roslaunch bitbots_bringup teamplayer.launch $ROBOT:=true motion:=false
            elif [[ $HOST == nuc* || $HOST == odroid* ]]; then
                roslaunch bitbots_bringup teamplayer.launch $ROBOT:=true motion:=false vision:=false
            elif [[ $HOST == jetson* ]]; then
                roslaunch bitbots_vision_common vision_startup.launch $ROBOT:=true
            fi
        fi
    )
    echo "You have two seconds to kill this script."
    sleep 2 || exit $?
done

