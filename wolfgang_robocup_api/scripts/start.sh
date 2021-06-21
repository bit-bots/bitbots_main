#!/bin/bash

############################################################################
# This script should be used to start the robocup stack for the simulator. #
# In the docker container, it is copied to /usr/local/bin/start. Therefore #
# it can be started using, for example, "start goalie 0" as the dockerCmd. #
############################################################################

#########
# Usage #
#########

function print_usage() {
    echo "Usage: $0 role position_number [--no-bag] [--team ID]"
    echo "       role: [offense | defense | goalie]"
    echo "       position_number: [0 | 1 | 2]"
}


if [[ -z $1 || -z $2 ]]; then
    print_usage
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

if [[ -z "$ROBOCUP_MIRROR_SERVER_IP" ]]; then
    echo "ROBOCUP_MIRROR_SERVER_IP is not set! Exiting."
    exit 2
fi

BRINGUP_DIR=$(rospack find bitbots_bringup)

if [[ -z "$BRINGUP_DIR" ]]; then
    echo "Could not find bitbots_bringup! Did you source ROS?"
    exit 2
fi

GAME_CONTROLLER_DIR=$(rospack find humanoid_league_game_controller)

if [[ -z "$GAME_CONTROLLER_DIR" ]]; then
    echo "Could not find humanoid_league_game_controller!"
    exit 2
fi

TEAM_COMM_DIR=$(rospack find humanoid_league_team_communication)

if [[ -z "$TEAM_COMM_DIR" ]]; then
    echo "Could not find humanoid_league_team_communication!"
    exit 2
fi

###########################
# Read command line flags #
###########################

ROLE=$1
shift

POSITION=$2
shift

RECORD=true

while test $# -gt 0; do
    case "$1" in
        -h|--help)
            print_usage
            exit 0
            ;;
        --no-bag)
            shift
            RECORD=false
            ;;
        --team)
            shift
            if test $# -gt 0; then
                if [[ "$1" =~ ^[0-9]+$ ]]; then
                    TEAM_ID=$1
                else
                    echo "Invalid team id: $1"
                    exit 1
                fi
            else
                echo "No team id for --team specified"
                exit 1
            fi
            shift
            ;;
        *)
            break
            ;;
    esac
done

#############################
# Write configuration files #
#############################

cat > $BRINGUP_DIR/config/game_settings.yaml << EOF
behavior/body/role_positions/pos_number: $POSITION
bot_id: $ROBOCUP_ROBOT_ID
role: $ROLE
team_color: $ROBOCUP_TEAM_COLOR
team_id: $TEAM_ID
EOF

cat > $GAME_CONTROLLER_DIR/config/game_controller.yaml << EOF
team_id: $TEAM_ID
bot_id: $ROBOCUP_ROBOT_ID
EOF

sed -i "/^target_host:/s/^.*$/target_host: $ROBOCUP_MIRROR_SERVER_IP/" $TEAM_COMM_DIR/config/team_communication_config.yaml

#############
# Start ROS #
#############

exec roslaunch wolfgang_robocup_api robocup_teamplayer.launch record:=$RECORD
