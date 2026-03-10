#!/usr/bin/env bash
set -eEuo pipefail

# static/global variables
DIR="$(dirname "$(readlink -f "$0")")"
BRANCH="${1:-main}"
REPO_URL_SSH="git@github.com:bit-bots/bitbots_main.git"
REPO_URL_HTTPS="https://github.com/bit-bots/bitbots_main.git"

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
        cd "$main_dir" || exit
        git checkout "$BRANCH"
    else
        if [[ ! -d "$PWD/bitbots_main" ]]; then
            echo "Cloning repository bitbots_main..."
            # Try to clone via SSH first, fall back to HTTPS if that fails
            if ! git clone "$REPO_URL_SSH"; then
                echo "SSH clone failed, trying HTTPS..."
                git clone "$REPO_URL_HTTPS"
            fi
            git checkout "$BRANCH"
        fi

        main_dir="$(realpath "$PWD/bitbots_main")"
        cd "$main_dir" || exit
    fi

    echo "Installing dependencies..."
    $HOME/.pixi/bin/pixi install
}

setup_host() {
    echo "Setting up system dependencies not covered by pixi. This may require sudo rights. For non-Ubuntu systems, please install the required packages manually."
    if (( has_sudo )); then
        $main_dir/scripts/make_basler.sh
        basler_installed=1
    fi
}

build_repository() {
    echo "Running full build..."
    set +u

    # Append "--packages-skip bitbots_basler_camera" to the build command if setup_host was skipped or failed
    if (( basler_installed )); then
    $HOME/.pixi/bin/pixi run build
    else
    $HOME/.pixi/bin/pixi run build --packages-skip bitbots_basler_camera
    fi
}

has_sudo=0
if ask_question "Do you have sudo rights?"; then
    has_sudo=1
fi
if (( ! has_sudo )); then
    echo "Because, you don't have sudo rights, no host dependencies will be installed."
fi

basler_installed=0

in_repo=1
main_dir="$(realpath "$DIR/../")"
if [[ ! -d "$main_dir/.git" ]]; then
    in_repo=0
fi

setup_pixi
setup_repo
setup_host
build_repository
