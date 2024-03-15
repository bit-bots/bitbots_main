#!/usr/bin/env bash
set -eEuo pipefail

# static/global variables
ROS_DISTRO=${ROS_DISTRO:-"iron"}
DIR="$(dirname "$(readlink -f "$0")")"
COLCON_WS="${COLCON_WS:-"$HOME/colcon_ws"}"
REPO_URL="git@github.com:bit-bots/bitbots_main.git"
SHELL_CONFIG="$(cat <<EOF

# >>> bit-bots initialize >>>

# Ignore some deprecation warnings
export PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources

# Limit ROS 2 communication to localhost (can be overridden when needed)
export ROS_DOMAIN_ID=24
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Set the default colcon workspace
export COLCON_WS="\$HOME/colcon_ws"

# Set the default log level for colcon
export COLCON_LOG_LEVEL=30

# Define a log layout
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

# Set the default Middleware
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Load our ros plugin script containing useful functions and aliases for ROS 2 development
if [[ -f \$COLCON_WS/src/bitbots_main/scripts/ros.plugin.sh ]]; then
  source \$COLCON_WS/src/bitbots_main/scripts/ros.plugin.sh
fi

# <<< bit-bots initialize <<<

EOF
)"

ask_question() {
    while true; do
        read -p "$1 [Y/n]: " -n 1 -r response
        echo ""
        case $response in
            [Yy] | "") return 0;;
            [Nn]) return 1;;
            * ) echo "Please answer yes or no.";;
        esac
    done
}

setup_repo() {
    echo "Setting up bitbots_main repository..."

    if (( in_repo )); then
        cd "$meta_dir" || exit
    else
        if [[ ! -d "$PWD/bitbots_main" ]]; then
            git clone "$REPO_URL"
        fi

        meta_dir="$(realpath "$PWD/bitbots_main")"
        cd "$meta_dir" || exit
    fi

    echo "Installing dependencies..."
    if (( has_sudo )); then
        make install
    else
        echo "Cannot use rosdep as it requires sudo."
        make install-no-root
    fi
}

setup_colcon_workspace() {
    if [[ ! -d "$COLCON_WS/src/bitbots_main" ]]; then
        echo "Setting up colcon workspace in $COLCON_WS..."
        mkdir -p "$COLCON_WS/src"
        ln -s "$meta_dir" "$COLCON_WS/src/bitbots_main"
    fi
}

build_repository() {
    echo "Running full colcon build for bitbots_main repository..."
    cd "$COLCON_WS"

    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"

    colcon build --symlink-install --continue-on-error
}

setup_shell_aliases() {
    if ask_question "Do you want to setup/update the bit-bots ros2/colcon shell configuration?"; then
        local shell_config_file

        if [[ "$SHELL" =~ "bash" ]]; then
            shell_config_file="$HOME/.bashrc"
        elif [[ "$SHELL" =~ "zsh" ]]; then
            shell_config_file="$HOME/.zshrc"
        else
            echo "Your shell is not supported!"
            exit 1
        fi

        if ! grep -q ">>> bit-bots initialize >>>" "$shell_config_file"; then
            echo "$SHELL_CONFIG" >> "$shell_config_file"
        fi
    fi
}

has_sudo=0
if ask_question "Do you have sudo rights?"; then
    has_sudo=1
fi
if (( ! has_sudo )); then
    echo "Because, you don't have sudo rights, you need ensure all necessary ros packages are preinstalled."
fi

in_repo=1
meta_dir="$(realpath "$DIR/../")"
if [[ ! -d "$meta_dir/.git" ]]; then
    in_repo=0
fi

setup_repo
setup_colcon_workspace
setup_shell_aliases
build_repository
