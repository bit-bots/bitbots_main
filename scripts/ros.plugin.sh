### Aliases and functions for ROS 2 and colcon usage. Usage for either
### Ubuntu 22.04 or in rosdocked/dev docker container

shell="$(basename "$SHELL")"

rid() {
  export ROS_DOMAIN_ID="$1"
  echo "ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"
}

# Create a function to update the argcomplete so tab completion works.
# This needs to be called every time we source something ROS 2 related.
# Previous loading of bashcompinit is required.
update_ros2_argcomplete() {
  eval "$(register-python-argcomplete3 colcon)"
  eval "$(register-python-argcomplete3 ros2)"
}

# Source the ROS 2 setup files if iron is installed
if [[ -d /opt/ros/iron ]]; then
  source "/opt/ros/iron/setup.$shell" &> /dev/null
fi

# Update the tab completion
update_ros2_argcomplete

# ros aliases
alias rr='ros2 run'
alias rl='ros2 launch'

alias rte='ros2 topic echo'
alias rtl='ros2 topic list'
alias rth='ros2 topic hz'
alias rtp='ros2 topic pub'

alias rpl='ros2 param list'
alias rpg='ros2 param get'

# colcon aliases
alias cdc='cd $COLCON_WS'

alias cba='cdc && colcon build --symlink-install --continue-on-error'
alias cbs='cdc && colcon build --symlink-install --packages-select'
alias cb='cdc && colcon build --symlink-install --continue-on-error --packages-up-to'
alias cc='cdc && colcon clean packages --packages-select'
alias cca='cdc && colcon clean packages'

alias sr="source /opt/ros/iron/setup.$shell && update_ros2_argcomplete"
alias sc="source \$COLCON_WS/install/setup.$shell && update_ros2_argcomplete"
alias sa='sr && sc'

# deploy_robots tool aliases
DEPLOY_ROBOTS="$COLCON_WS/src/bitbots_main/scripts/deploy_robots.py"
alias dp='$DEPLOY_ROBOTS --sync --build --print-bit-bot'
alias dpfull='dp --install --configure'
alias dpclean='dp --clean'
alias dplo='dp --skip-local-repo-check'
