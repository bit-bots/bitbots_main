#!/bin/bash

ROBOT_IP = ${1?"Usage: bridge.sh <robot-ip>"}

export ZENOH_CONFIG=${mktemp /tmp/zenoh_bridge_config.json5}

cat > $ZENOH_CONFIG <<CONTENT
{
  mode: "peer",
  connect: {
    endpoints: ["tcp/${ROBOT_IP}:7447"]
  }
}
CONTENT

pixi run ros2 launch bitbots_emergency emergency.launch remote:=true
