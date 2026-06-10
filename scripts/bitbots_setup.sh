#!/usr/bin/env bash
set -Eeuo pipefail

readonly DEFAULT_BRANCH="main"
readonly REPO_URL_SSH="git@github.com:bit-bots/bitbots_main.git"
readonly REPO_URL_HTTPS="https://github.com/bit-bots/bitbots_main.git"

branch="$DEFAULT_BRANCH"
branch_explicit=false
clone_method="ssh"
clone_method_explicit=false

usage() {
    echo "Usage: $0 [--ssh | --https] [branch]" >&2
}

parse_args() {
    for arg in "$@"; do
        case "$arg" in
            --ssh | --https)
                local method="${arg#--}"
                if $clone_method_explicit && [[ "$clone_method" != "$method" ]]; then
                    echo "Only one clone method can be selected." >&2
                    usage
                    exit 2
                fi
                clone_method="$method"
                clone_method_explicit=true
                ;;
            -*)
                echo "Unknown option: $arg" >&2
                usage
                exit 2
                ;;
            *)
                if $branch_explicit; then
                    echo "Only one branch can be selected." >&2
                    usage
                    exit 2
                fi
                branch="$arg"
                branch_explicit=true
                ;;
        esac
    done
}

ask_question() {
    local response

    while true; do
        read -r -p "$1 [Y/n]: " response
        case "$response" in
            [Yy] | [Yy][Ee][Ss] | "") return 0 ;;
            [Nn] | [Nn][Oo]) return 1 ;;
            *) echo "Please answer yes or no." ;;
        esac
    done
}

setup_pixi() {
    command -v pixi >/dev/null && return

    command -v curl >/dev/null || {
        echo "Required command not found: curl" >&2
        exit 1
    }
    echo "Installing Pixi..."
    curl -fsSL https://pixi.sh/install.sh | bash
    export PATH="$HOME/.pixi/bin:$PATH"
    command -v pixi >/dev/null || {
        echo "Pixi installation completed, but pixi is not available on PATH." >&2
        exit 1
    }
}

clone_repo() {
    local target="$PWD/bitbots_main"

    if [[ -e "$target" ]]; then
        git -C "$target" rev-parse --is-inside-work-tree >/dev/null 2>&1 || {
            echo "Cannot clone: $target already exists and is not a Git repository." >&2
            exit 1
        }
        git -C "$target" switch "$branch"
        return
    fi

    echo "Cloning bitbots_main branch '$branch'..."
    if [[ "$clone_method" == "https" ]]; then
        git clone --branch "$branch" --single-branch "$REPO_URL_HTTPS" "$target"
    elif git clone --branch "$branch" --single-branch "$REPO_URL_SSH" "$target"; then
        :
    else
        echo "SSH clone failed. This may mean your SSH keys are not set up for GitHub."
        echo "See: https://docs.github.com/en/authentication/connecting-to-github-with-ssh"
        if [[ ! -t 0 ]]; then
            echo "Cannot ask to retry with HTTPS because standard input is not interactive." >&2
            echo "Re-run the script with --https." >&2
            exit 1
        fi
        if ask_question "Continue with an HTTPS clone instead?"; then
            git clone --branch "$branch" --single-branch "$REPO_URL_HTTPS" "$target"
        else
            echo "Set up your SSH keys and re-run this script." >&2
            exit 1
        fi
    fi
}

main() {
    parse_args "$@"

    command -v git >/dev/null || {
        echo "Required command not found: git" >&2
        exit 1
    }
    setup_pixi

    clone_repo

    cd "$PWD/bitbots_main"
    echo "Installing dependencies..."
    pixi install
    echo "Running full build..."
    pixi run -e default build --parallel-workers 2
}

main "$@"
