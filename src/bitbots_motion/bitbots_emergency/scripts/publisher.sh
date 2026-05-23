#!/bin/bash

ROBOT_IP=${1?"Usage: publisher.sh <robot-ip>"}

export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${ROBOT_IP}:7447\"]"
