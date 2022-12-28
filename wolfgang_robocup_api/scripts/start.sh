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

UTILS_DIR=$(colcon list --paths-only --packages-select bitbots_utils)

if [[ -z "$BRINGUP_DIR" ]]; then
    echo "Could not find bitbots_utils! Did you source ROS?"
    exit 2
fi

TEAM_COMM_DIR=$(colcon list --paths-only --packages-select humanoid_league_team_communication)

if [[ -z "$TEAM_COMM_DIR" ]]; then
    echo "Could not find humanoid_league_team_communication!"
    exit 2
fi

###########################
# Read command line flags #
###########################

ROLE=$1
shift

POSITION=$1
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

cat > $UTILS_DIR/config/game_settings.yaml << EOF
parameter_blackboard:
  ros__parameters:
    bot_id: $ROBOCUP_ROBOT_ID
    position_number: $POSITION
    role: $ROLE
    team_color: $ROBOCUP_TEAM_COLOR
    team_id: $TEAM_ID
EOF

sed -i "/^    target_host:/s/^.*$/    target_host: $ROBOCUP_MIRROR_SERVER_IP/" $TEAM_COMM_DIR/config/team_communication_config.yaml

#############
# Start ROS #
#############

exec ros2 launch wolfgang_robocup_api robocup_teamplayer.lanuch record:=$RECORD
