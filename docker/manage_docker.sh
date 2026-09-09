#!/bin/bash

# Hamburg Bit-Bots Docker Management Script
# This script helps building and running the Docker images.

set -e

# Configuration
IMAGE_NAME_COMMON="bitbots-common"
IMAGE_NAME_PROJECT="bitbots-project"
IMAGE_NAME_TARGET="bitbots-target"
DEFAULT_USER="bitbots"
SSH_PORT_PROJECT=2222
SSH_PORT_TARGET=2223
NETWORK_NAME="bitbots-net"
DEFAULT_SUBNET="10.66.0.0/16"

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Function to find SSH public key
find_ssh_key() {
    if [ -n "$SSH_PUB_KEY_PATH" ] && [ -f "$SSH_PUB_KEY_PATH" ]; then
        echo "$SSH_PUB_KEY_PATH"
        return
    fi

    local key_path=""
    if [ -f "$HOME/.ssh/id_ed25519.pub" ]; then
        key_path="$HOME/.ssh/id_ed25519.pub"
    elif [ -f "$HOME/.ssh/id_rsa.pub" ]; then
        key_path="$HOME/.ssh/id_rsa.pub"
    fi
    echo "$key_path"
}

# Function to resolve robot IP from identifier
resolve_robot_ip() {
    local identifier=$1
    if [ -z "$identifier" ]; then
        return
    fi

    local ip
    ip=$(python3 -c "
import yaml
import sys
import ipaddress
try:
    with open('$REPO_ROOT/scripts/deploy/known_targets.yaml', 'r') as f:
        data = yaml.safe_load(f)
    ident = '$identifier'.lower()
    net = ipaddress.ip_network('$DEFAULT_SUBNET')
    
    matches = []
    for ip_str, info in data.items():
        if info.get('hostname', '').lower() == ident or info.get('robot_name', '').lower() == ident or ip_str == ident:
            matches.append(ip_str)
    
    # Prefer IP in our subnet
    found_ip = None
    for m in matches:
        try:
            if ipaddress.ip_address(m) in net:
                found_ip = m
                break
        except ValueError:
            pass
    
    if not found_ip and matches:
        found_ip = matches[0]
        
    if found_ip:
        print(found_ip)
except Exception as e:
    sys.stderr.write(f'Error parsing YAML: {e}\n')
")

    if [ -n "$ip" ]; then
        echo "$ip"
    elif [[ $identifier =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo "$identifier"
    fi
}

# Function to get GPU flags for container run
get_gpu_args() {
    if [ -n "$GPU_ARGS" ]; then
        echo "$GPU_ARGS"
        return
    fi

    local args=()
    if [ -d /dev/dri ]; then
        args+=(--device /dev/dri)
    fi
    if [ -e /dev/kfd ]; then
        args+=(--device /dev/kfd)
    fi
    if command -v nvidia-smi >/dev/null 2>&1 || [ -e /dev/nvidia0 ] || [ -e /dev/nvidiactl ]; then
        args+=(--gpus all)
    fi
    echo "${args[@]}"
}

show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  build-common   Build the common base image"
    echo "  build-project  Build the project image"
    echo "  build-target   Build the target image"
    echo "  build-all      Build all three images"
    echo "  create-network [subnet]  Create Docker network (default: $DEFAULT_SUBNET)"
    echo "  run-project [id]         Run project container (uses port $SSH_PORT_PROJECT or specified IP/robot name)"
    echo "  run-target [id]          Run target container (uses port $SSH_PORT_TARGET or specified IP/robot name)"
    echo "  ssh <id>                 SSH into a running container (handles host keys and networking automatically)"
    echo "  net-shell                Enter network namespace shell"
    echo "  stop-all                 Stop all Bit-Bots containers"
    echo "  help                     Show this help message"
    echo ""
    echo "Environment Variables:"
    echo "  SSH_PUB_KEY_PATH         Path to your SSH public key (default: auto-detect)"
    echo "  GPU_ARGS                 Override GPU flags passed to container (default: auto-detect)"
    echo ""
    echo "Note: To make container IPs reachable from the host, ensure the Docker bridge network is accessible or use SSH port mapping."
}

build_common() {
    local key_path=$(find_ssh_key)
    local build_args=()

    if [ -n "$key_path" ]; then
        echo "Using SSH key from $key_path"
        build_args+=(--build-arg "ssh_pub_key=$(cat "$key_path")")
    else
        echo "Warning: No SSH public key found in ~/.ssh/id_ed25519.pub or ~/.ssh/id_rsa.pub"
        echo "SSH access will not be possible without manual configuration."
    fi

    echo "Building common base image..."
    docker build -t "$IMAGE_NAME_COMMON" \
        "${build_args[@]}" \
        -f "$SCRIPT_DIR/Containerfile.common" \
        "$REPO_ROOT"
}

build_project() {
    echo "Building project image..."
    docker build -t "$IMAGE_NAME_PROJECT" \
        --build-arg BASE_IMAGE="$IMAGE_NAME_COMMON" \
        -f "$SCRIPT_DIR/Containerfile.project" \
        "$REPO_ROOT"
}

build_target() {
    echo "Building target image..."
    docker build -t "$IMAGE_NAME_TARGET" \
        --build-arg BASE_IMAGE="$IMAGE_NAME_COMMON" \
        -f "$SCRIPT_DIR/Containerfile.target" \
        "$REPO_ROOT"
}

create_network() {
    local subnet=${1:-$DEFAULT_SUBNET}
    if ! docker network inspect "$NETWORK_NAME" >/dev/null 2>&1; then
        echo "Creating network $NETWORK_NAME with subnet $subnet..."
        docker network create --subnet "$subnet" "$NETWORK_NAME"
    else
        echo "Network $NETWORK_NAME already exists."
    fi
}

run_project() {
    local target=$1
    local ip=""
    local name="bitbots-project-run"
    local net_args=()

    if [ -n "$target" ]; then
        ip=$(resolve_robot_ip "$target")
        if [ -z "$ip" ]; then
            echo "Error: Could not find IP for target: $target"
            exit 1
        fi
        name="bitbots-project-${ip//./-}"
        net_args=(--network "$NETWORK_NAME" --ip "$ip")
        create_network
        echo "Running project container $name with IP $ip..."
        echo "You can connect via: ssh $DEFAULT_USER@$ip"
    else
        net_args=(-p "$SSH_PORT_PROJECT:22")
        echo "Running project container $name on port $SSH_PORT_PROJECT..."
        echo "You can connect via: ssh -p $SSH_PORT_PROJECT $DEFAULT_USER@localhost"
    fi

    local gpu_args=($(get_gpu_args))

    docker run -d --name "$name" \
        "${net_args[@]}" \
        "${gpu_args[@]}" \
        "$IMAGE_NAME_PROJECT"
}

run_target() {
    local target=$1
    local ip=""
    local name="bitbots-target-run"
    local net_args=()

    if [ -n "$target" ]; then
        ip=$(resolve_robot_ip "$target")
        if [ -z "$ip" ]; then
            echo "Error: Could not find IP for target: $target"
            exit 1
        fi
        name="bitbots-target-${ip//./-}"
        net_args=(--network "$NETWORK_NAME" --ip "$ip")
        create_network
        echo "Running target container $name with IP $ip..."
        echo "You can connect via: ssh $DEFAULT_USER@$ip"
    else
        net_args=(-p "$SSH_PORT_TARGET:22")
        echo "Running target container $name on port $SSH_PORT_TARGET..."
        echo "You can connect via: ssh -p $SSH_PORT_TARGET $DEFAULT_USER@localhost"
    fi

    local gpu_args=($(get_gpu_args))

    docker run -d --name "$name" \
        "${net_args[@]}" \
        "${gpu_args[@]}" \
        "$IMAGE_NAME_TARGET"
}

stop_all() {
    echo "Stopping Bit-Bots containers..."
    local containers=$(docker ps -a --format "{{.Names}}" | grep "^bitbots-")
    if [ -n "$containers" ]; then
        docker stop $containers
        docker rm $containers
    else
        echo "No Bit-Bots containers found."
    fi
}

net_shell() {
    if ! command -v docker >/dev/null; then
        echo "Error: docker not found."
        exit 1
    fi

    echo "Entering Docker network environment..."
    if [ -f "$XDG_RUNTIME_DIR/docker.netns" ]; then
        echo "Entering rootless Docker network namespace..."
        nsenter --net="$XDG_RUNTIME_DIR/docker.netns" bash
    elif [ -f "$XDG_RUNTIME_DIR/docker.pid" ] && command -v nsenter >/dev/null 2>&1; then
        local pid=$(cat "$XDG_RUNTIME_DIR/docker.pid")
        echo "Entering rootless Docker network namespace (PID: $pid)..."
        nsenter -t "$pid" -n bash
    else
        echo "---------------------------------------------------------"
        echo "  BIT-BOTS NETWORK SHELL"
        echo "---------------------------------------------------------"
        echo "In Docker, containers on '$NETWORK_NAME' are directly reachable"
        echo "from the host network namespace (e.g., mickey at 10.66.6.2)."
        echo "Type 'exit' to return to your normal shell."
        echo "---------------------------------------------------------"
        export BITBOTS_NET_SHELL=1
        ${SHELL:-bash}
    fi
}

ssh_container() {
    local target=$1
    if [ -z "$target" ]; then
        echo "Usage: $0 ssh <hostname|robot_name|IP>"
        exit 1
    fi

    local ip=$(resolve_robot_ip "$target")
    if [ -z "$ip" ]; then
        echo "Error: Could not find IP for target: $target"
        exit 1
    fi

    # Common SSH options to prevent host key warnings for the virtual network
    local ssh_opts=(-o "StrictHostKeyChecking=no" -o "UserKnownHostsFile=/dev/null" -o "LogLevel=ERROR")

    if [ "$BITBOTS_NET_SHELL" = "1" ] || [ "$(id -u)" = "0" ] || ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
        # Direct connection if in net-shell, running as root, or if IP is directly reachable
        ssh "${ssh_opts[@]}" "bitbots@$ip"
    else
        # Try to find container ID for proxying in case IP is not directly routed
        # Match by name which contains the IP (e.g., bitbots-target-10-66-6-2)
        local ip_slug=${ip//./-}
        local cid=$(docker ps --format "{{.ID}} {{.Names}}" | grep "bitbots" | grep "$ip_slug" | head -n 1 | cut -d' ' -f1)
        if [ -n "$cid" ]; then
            ssh "${ssh_opts[@]}" \
                -o "ProxyCommand=docker exec -i $cid nc localhost 22" \
                "bitbots@$ip"
        else
            echo "Error: Container with IP $ip not found or not running."
            echo "Ensure the container is started and attached to '$NETWORK_NAME'."
            exit 1
        fi
    fi
}

case "$1" in
    build-common)
        build_common
        ;;
    build-project)
        build_project
        ;;
    build-target)
        build_target
        ;;
    build-all)
        build_common
        build_project
        build_target
        ;;
    run-target)
        run_target "$2"
        ;;
    run-project)
        run_project "$2"
        ;;
    create-network)
        create_network "$2"
        ;;
    stop-all)
        stop_all
        ;;
    ssh)
        ssh_container "$2"
        ;;
    net-shell)
        net_shell
        ;;
    help|*)
        show_help
        ;;
esac
