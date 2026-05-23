#!/bin/bash

ROBOT_IP=${1?"Usage: publisher.sh <robot-ip>"}

export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${ROBOT_IP}:7447\"]"

pixi run ros2 run bitbots_emergency EMERGENCY_NODE_PUBLISHER
