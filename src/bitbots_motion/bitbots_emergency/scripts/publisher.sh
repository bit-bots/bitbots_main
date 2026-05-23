#!/bin/bash

#ROBOT_IP=${1?"Usage: publisher.sh <robot-ip>"}

export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/${10.222.10.206}:7447"]'

pixi run ros2 launch bitbots_emergency emergency.launch remote:=true
