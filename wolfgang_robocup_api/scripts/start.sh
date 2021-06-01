#!/bin/bash

############################################################################
# This script should be used to start the robocup stack for the simulator. #
# In the docker container, it is copied to /usr/local/bin/start. Therefore #
# it can be started using, for example, "start goalie 0" as the dockerCmd. #
############################################################################

#########
# Usage #
#########

if [[ -z $1 || -z $2 ]]; then
    echo "Usage: $0 role position_number [--no-bag]"
    echo "       role: [offense | defense | goalie]"
    echo "       position_number: [0 | 1 | 2]"
    exit 1
fi

#################
# Configuration #
#################

TEAM_ID=7

##########
# Checks #
##########

if [[ -z "$ROBOCUP_ROBOT_ID" ]]; then
    echo "ROBOCUP_ROBOT_ID is not set! Exiting."
    exit 2
fi

if [[ -z "$ROBOCUP_TEAM_COLOR" ]]; then
    echo "ROBOCUP_TEAM_COLOR is not set! Exiting."
    exit 2
fi

if [[ -z "$ROBOCUP_SIMULATOR_ADDR" ]]; then
    echo "ROBOCUP_SIMULATOR_ADDR is not set! Exiting."
    exit 2
fi

BRINGUP_DIR=$(rospack find bitbots_bringup)

if [[ -z $BRINGUP_DIR ]]; then
    echo "Could not find bitbots_bringup! Did you source ROS?"
    exit 2
fi

GAME_CONTROLLER_DIR=$(rospack find humanoid_league_game_controller)

if [[ -z $BRINGUP_DIR ]]; then
    echo "Could not find humanoid_league_game_controller!"
    exit 2
fi

#############################
# Write configuration files #
#############################

cat > $BRINGUP_DIR/config/game_settings.yaml << EOF
behavior/body/role_positions/pos_number: $2
bot_id: $ROBOCUP_ROBOT_ID
role: $1
team_color: $ROBOCUP_TEAM_COLOR
team_id: $TEAM_ID
EOF

cat > $GAME_CONTROLLER_DIR/config/game_controller.yaml << EOF
team_id: $TEAM_ID
bot_id: $ROBOCUP_ROBOT_ID
EOF

#############
# Start ROS #
#############

if [[ "$3" == "--no-bag" ]]; then
    RECORD=false
else
    RECORD=true
fi

exec roslaunch wolfgang_robocup_api robocup_teamplayer.launch record:=$RECORD
