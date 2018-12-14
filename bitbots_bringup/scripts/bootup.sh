#!/usr/bin/env zsh

if [ "$TERM" != "screen" ] ; then
    echo "this script must be run with screen"
    exit 1
fi

source ~/boot-configuration.sh
source $WORKSPACE/devel/setup.sh

if $AUTOSTART; then
    screen $WORKSPACE/src/bitbots_misc/bitbots_bringup/scripts/boot-roscore.zsh
    sleep 1
    screen $WORKSPACE/src/bitbots_misc/bitbots_bringup/scripts/boot-highlevel.zsh
    sleep 5
    screen $WORKSPACE/src/bitbots_misc/bitbots_bringup/scripts/boot-robot-control.zsh
fi
