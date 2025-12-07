#!/usr/bin/env bash
set -eEuo pipefail

# static/global variables
DIR="$(dirname "$(readlink -f "$0")")"
BRANCH="${1:-main}"
REPO_URL="git@github.com:bit-bots/bitbots_main.git"

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

setup_pixi() {
    curl -fsSL https://pixi.sh/install.sh | sh
}

setup_repo() {
    echo "Setting up bitbots_main repository..."

    if (( in_repo )); then
        cd "$meta_dir" || exit
        git checkout "$BRANCH"
    else
        if [[ ! -d "$PWD/bitbots_main" ]]; then
            git clone "$REPO_URL"
            git checkout "$BRANCH"
        fi

        meta_dir="$(realpath "$PWD/bitbots_main")"
        cd "$meta_dir" || exit
    fi

    echo "Installing dependencies..."
    $HOME/.pixi/bin/pixi install
}

setup_host() {
    echo "Setting up system dependencies not covered by pixi. This may require sudo rights. For non-Ubuntu systems, please install the required packages manually."
    if (( has_sudo )); then
        $meta_dir/scripts/make_basler.sh
        $meta_dir/scripts/make_webots.sh
    fi
}

build_repository() {
    echo "Running full colcon build..."
    set +u
    $HOME/.pixi/bin/pixi run build
}

has_sudo=0
if ask_question "Do you have sudo rights?"; then
    has_sudo=1
fi
if (( ! has_sudo )); then
    echo "Because, you don't have sudo rights, no host dependencies will be installed."
fi

in_repo=1
meta_dir="$(realpath "$DIR/../")"
if [[ ! -d "$meta_dir/.git" ]]; then
    in_repo=0
fi

setup_ros
setup_repo
setup_host
build_repository
