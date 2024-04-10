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

setup_ros() {
    if (( has_sudo )); then
        echo "Setting up ROS 2..."

        if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
            echo "Adding ROS 2 repository..."
            sudo apt install -y curl lsb-release
            curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key > /usr/share/keyrings/ros-archive-keyring.gpg
            sudo sh -c "echo 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main' > /etc/apt/sources.list.d/ros2.list"
        fi

        sudo apt update && sudo apt install -y \
            clang-format \
            cppcheck \
            python3-colcon-clean \
            python3-colcon-common-extensions \
            python3-pip \
            python3-rosdep \
            python3-vcstool \
            "ros-${ROS_DISTRO}-desktop-full" \
            "ros-${ROS_DISTRO}-plotjuggler-ros" \
            "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
            "ros-${ROS_DISTRO}-rqt-robot-monitor" \
            "ros-${ROS_DISTRO}-rqt-runtime-monitor"
    else
        echo "Please install ROS 2 manually!"
    fi
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

setup_colcon() {
    echo "Installing/Updating colcon extensions..."

    # Install/Update colcon extensions / patches
    python3 -m pip install --upgrade --user \
      git+https://github.com/timonegk/colcon-core.git@colors \
      git+https://github.com/timonegk/colcon-notification.git@colors \
      git+https://github.com/timonegk/colcon-output.git@colors

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
    if ask_question "Do you want to setup/update the Bit-Bots ROS 2 and colcon shell configuration?"; then
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
    echo "Because, you don't have sudo rights, ensure all necessary ROS 2 packages are installed."
fi

in_repo=1
meta_dir="$(realpath "$DIR/../")"
if [[ ! -d "$meta_dir/.git" ]]; then
    in_repo=0
fi

setup_ros
setup_repo
setup_colcon
setup_shell_aliases
build_repository
