### Aliases and functions for ROS 2 and colcon usage.

rid() {
  export ROS_DOMAIN_ID="$1"
  echo "ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"
}

# Create a function to update the argcomplete and generate the completion
# scripts so that tab completion works within `pixi shell` environments.
update_argcompletes() {
  local tools=(
      ros2
      colcon
  )

  if [[ -n "${CONDA_PREFIX:-}" ]]; then
    local completions_path="$CONDA_PREFIX/share/zsh/site-functions"

    if ! [[ -d "$completions_path" ]]; then
        mkdir -p "$completions_path"
    fi

    for tool in "${tools[@]}"; do
        if type "$tool" &> /dev/null; then
          local _completion="$(register-python-argcomplete "$tool")"

          if ! [[ -f "$completions_path/_$tool" ]]; then
            echo "_completion" > "$completions_path/_$tool"
          fi
        fi
    done
  fi
}

setup_alises() {
  # check if we are in subdir of $ROS_WORKSPACE and switch to it otherwise
  alias cdc='[[ "$PWD" = "$ROS_WORKSPACE"* ]] || cd "$ROS_WORKSPACE"'

  alias psh='cdc && pixi shell'

  # ros aliases
  alias ros2='cdc && pixi run ros2'
  alias rr='ros2 run'
  alias rl='ros2 launch'

  alias rte='ros2 topic echo'
  alias rtl='ros2 topic list'
  alias rth='ros2 topic hz'
  alias rtp='ros2 topic pub'

  alias rpl='ros2 param list'
  alias rpg='ros2 param get'

  # colcon aliases
  alias colcon='cdc && pixi run colcon'
  alias cba='cdc && pixi run build'
  alias cbs='cba --packages-select'
  alias cb='cba --packages-up-to'

  alias ct='pixi run test'
  alias cts='ct --packages-select'

  alias cca='pixi run clean'

  # deploy_robots tool aliases
  alias dp='pixi run deploy --sync --build --print-bit-bot'
  alias dpfull='dp --install --configure'
  alias dpclean='dp --clean'
  alias dplo='dp --skip-local-repo-check'

  # Overwrite some aliases in pixi shell to allow for tab completion
  # by directly using ros2/colcon instead of the pixi tasks
  if [[ "$(ps -o comm= -p "$PPID")" == "pixi" ]]; then
    alias cdc='cd $PIXI_PROJECT_ROOT'

    # ros aliases
    unalias ros2
    alias rr='ros2 run'
    alias rl='ros2 launch'

    alias rte='ros2 topic echo'
    alias rtl='ros2 topic list'
    alias rth='ros2 topic hz'
    alias rtp='ros2 topic pub'

    alias rpl='ros2 param list'
    alias rpg='ros2 param get'

    # colcon aliases
    unalias colcon
    alias cba='cdc && colcon build --symlink-install --cmake-args -G "Unix Makefiles" --continue-on-error'
    alias cbs='cba --packages-select'
    alias cb='cba --packages-up-to'

    alias ct='cdc && colcon test --event-handlers console_direct+ --return-code-on-test-failure'
    alias cts='ct --packages-select'
  fi
}

update_argcompletes
setup_alises
