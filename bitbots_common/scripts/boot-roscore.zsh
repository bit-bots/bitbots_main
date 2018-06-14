#!/bin/zsh

source ~/boot-configuration.sh
source $WORKSPACE/devel/setup.zsh

if [[ $ROBOT != "wolfgang" || $HOST == nuc* ]]; then
    roscore
fi
