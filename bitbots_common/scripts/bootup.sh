#!/usr/bin/env bash

if [ "$TERM" != "screen" ] ; then
    echo "this script must be run with screen"
    exit 1
fi

source ~/catkin_ws/devel/setup.sh

screen roscore
sleep 1
screen ~/catkin_ws/src/bitbots_meta/bitbots_misc/bitbots_common/scripts/boot-robot-control.sh
screen ~/catkin_ws/src/bitbots_meta/bitbots_misc/bitbots_common/scripts/boot-highlevel.sh