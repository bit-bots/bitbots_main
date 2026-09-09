#!/bin/bash

# Hamburg Bit-Bots Docker Management Script
# This script helps building and running the Docker images with support for
# single-host and multi-PC Docker Swarm overlay networks.

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
    echo "  build-common             Build the common base image"
    echo "  build-project            Build the project image"
    echo "  build-target             Build the target image"
    echo "  build-all                Build all three images"
    echo "  swarm-init [addr]        Initialize Docker Swarm on manager PC (optional advertise-addr)"
    echo "  swarm-join [args]        Join a Docker Swarm cluster from another PC"
    echo "  swarm-leave              Leave the current Docker Swarm cluster"
    echo "  create-network [subnet]  Create attachable overlay network for multi-PC networking (default: $DEFAULT_SUBNET)"
    echo "  connect-host [ip]        Connect host directly to overlay network (default: 10.66.0.254/16, requires sudo)"
    echo "  disconnect-host          Disconnect host interface from overlay network"
    echo "  run-project [id]         Run project container (uses port $SSH_PORT_PROJECT or specified IP/robot name)"
    echo "  run-target [id]          Run target container (uses port $SSH_PORT_TARGET or specified IP/robot name)"
    echo "  ssh <id>                 SSH into a running container (handles host keys and networking automatically)"
    echo "  net-shell                Enter network namespace shell"
    echo "  stop-all                 Stop all Bit-Bots containers and clean up host interface"
    echo "  help                     Show this help message"
    echo ""
    echo "Environment Variables:"
    echo "  SSH_PUB_KEY_PATH         Path to your SSH public key (default: auto-detect)"
    echo "  GPU_ARGS                 Override GPU flags passed to container (default: auto-detect)"
    echo ""
    echo "Note: For multi-PC container networking, initialize Docker Swarm with 'swarm-init' on the manager PC and join other PCs with 'swarm-join'."
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

swarm_init() {
    local advertise_addr=$1
    local init_args=()
    if [ -n "$advertise_addr" ]; then
        init_args+=(--advertise-addr "$advertise_addr")
    fi

    local swarm_state
    swarm_state=$(docker info --format '{{.Swarm.LocalNodeState}}' 2>/dev/null || echo "inactive")
    if [ "$swarm_state" = "active" ]; then
        echo "Docker Swarm is already active on this node."
    else
        echo "Initializing Docker Swarm..."
        docker swarm init "${init_args[@]}"
    fi

    echo ""
    echo "To join other PCs to this swarm as worker nodes, run on those PCs:"
    docker swarm join-token worker 2>/dev/null || true
    echo ""
    echo "To create the shared multi-host overlay network, run:"
    echo "  $0 create-network"
}

swarm_join() {
    if [ $# -eq 0 ]; then
        echo "Usage: $0 swarm-join <token> <manager-ip:port>"
        echo "   or: $0 swarm-join --token <token> <manager-ip:port>"
        exit 1
    fi

    if [[ "$1" == "--token" ]]; then
        docker swarm join "$@"
    elif [ $# -eq 2 ]; then
        docker swarm join --token "$1" "$2"
    else
        docker swarm join "$@"
    fi
}

swarm_leave() {
    echo "Leaving Docker Swarm..."
    docker swarm leave --force
}

create_network() {
    local subnet=${1:-$DEFAULT_SUBNET}

    local swarm_state
    swarm_state=$(docker info --format '{{.Swarm.LocalNodeState}}' 2>/dev/null || echo "inactive")
    local is_manager
    is_manager=$(docker info --format '{{.Swarm.ControlAvailable}}' 2>/dev/null || echo "false")

    if [ "$swarm_state" != "active" ]; then
        echo "Docker Swarm is not active. Initializing Docker Swarm..."
        if ! docker swarm init >/dev/null 2>&1; then
            echo "Auto-init failed. Running 'docker swarm init' with interactive output:"
            docker swarm init || {
                echo "Error: Failed to initialize Docker Swarm. If you have multiple network interfaces, run: $0 swarm-init <advertise_addr>"
                exit 1
            }
        else
            echo "Docker Swarm initialized successfully."
        fi
        is_manager="true"
    fi

    if [ "$is_manager" = "false" ]; then
        echo "Connected to Docker Swarm as worker node. Overlay network '$NETWORK_NAME' is managed by the Swarm manager."
        return 0
    fi

    if docker network inspect "$NETWORK_NAME" >/dev/null 2>&1; then
        local driver
        driver=$(docker network inspect -f '{{.Driver}}' "$NETWORK_NAME" 2>/dev/null || true)
        if [ "$driver" = "overlay" ]; then
            echo "Attachable overlay network '$NETWORK_NAME' already exists."
            return 0
        else
            echo "Network '$NETWORK_NAME' exists with driver '$driver'. Recreating as attachable overlay network..."
            docker network rm "$NETWORK_NAME"
        fi
    fi

    echo "Creating attachable overlay network $NETWORK_NAME with subnet $subnet..."
    docker network create --driver overlay --attachable --subnet "$subnet" "$NETWORK_NAME"
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
    if ip link show "veth-bb-host" >/dev/null 2>&1; then
        echo "Removing host network interface veth-bb-host..."
        if [ "$(id -u)" -ne 0 ]; then
            sudo ip link delete "veth-bb-host" 2>/dev/null || true
        else
            ip link delete "veth-bb-host" 2>/dev/null || true
        fi
    fi
}

# Helper to find overlay network namespace file
find_overlay_netns() {
    local net_id
    net_id=$(docker network inspect -f '{{.Id}}' "$NETWORK_NAME" 2>/dev/null || true)
    if [ -z "$net_id" ]; then
        return 1
    fi

    # Script executed with root privileges if needed to access /var/run/docker/netns (mode 0700)
    local search_script='
        net_id="$1"
        dirs=("/var/run/docker/netns" "/run/docker/netns" "/var/run/netns" "/run/netns")

        # 1. Search for matching network ID prefix (e.g. 1-<id_prefix> or <id_prefix>)
        for dir in "${dirs[@]}"; do
            [ -d "$dir" ] || continue
            for len in 9 10 8 12 16 64; do
                short="${net_id:0:$len}"
                [ -n "$short" ] || continue
                if [ -f "$dir/1-$short" ]; then
                    echo "$dir/1-$short"
                    exit 0
                fi
                if [ -f "$dir/$short" ]; then
                    echo "$dir/$short"
                    exit 0
                fi
            done
            # Substring match on net_id prefix (excluding lb_ loadbalancer namespace)
            short8="${net_id:0:8}"
            if [ -n "$short8" ]; then
                for f in "$dir"/*"$short8"*; do
                    if [ -f "$f" ] && [[ "$(basename "$f")" != lb_* ]]; then
                        echo "$f"
                        exit 0
                    fi
                done
            fi
        done

        # 2. Look for any 1-* overlay namespace containing bridge br0
        for dir in "${dirs[@]}"; do
            [ -d "$dir" ] || continue
            for f in "$dir"/1-*; do
                if [ -f "$f" ] && nsenter --net="$f" ip link show br0 >/dev/null 2>&1; then
                    echo "$f"
                    exit 0
                fi
            done
        done

        exit 1
    '

    if [ "$(id -u)" -eq 0 ]; then
        bash -c "$search_script" -- "$net_id" 2>/dev/null
    else
        sudo bash -c "$search_script" -- "$net_id" 2>/dev/null
    fi
}

_do_connect_host() {
    local cpid=$1
    local ifname=$2
    local peername=$3

    # Remove existing interface on host if present
    if ip link show "$ifname" >/dev/null 2>&1; then
        ip link delete "$ifname" 2>/dev/null || true
    fi

    # Create veth pair
    ip link add "$ifname" type veth peer name "$peername"

    # Move peer into the container's network namespace
    ip link set "$peername" netns "$cpid"

    # Configure transit link inside container
    nsenter -t "$cpid" -n ip link set lo up 2>/dev/null || true
    nsenter -t "$cpid" -n ip link set "$peername" up
    nsenter -t "$cpid" -n ip addr add 10.66.254.2/30 dev "$peername"

    # Configure IP forwarding and iptables masquerade inside container
    nsenter -t "$cpid" -n sysctl -w net.ipv4.ip_forward=1 >/dev/null 2>&1 || true
    nsenter -t "$cpid" -n iptables -t nat -C POSTROUTING -o eth0 -j MASQUERADE 2>/dev/null || \
        nsenter -t "$cpid" -n iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE 2>/dev/null || true
    nsenter -t "$cpid" -n iptables -C FORWARD -i "$peername" -o eth0 -j ACCEPT 2>/dev/null || \
        nsenter -t "$cpid" -n iptables -A FORWARD -i "$peername" -o eth0 -j ACCEPT 2>/dev/null || true
    nsenter -t "$cpid" -n iptables -C FORWARD -i eth0 -o "$peername" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || \
        nsenter -t "$cpid" -n iptables -A FORWARD -i eth0 -o "$peername" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true

    # Configure host side
    ip link set "$ifname" up
    ip addr add 10.66.254.1/30 dev "$ifname"
    ip route replace 10.66.0.0/16 via 10.66.254.2 dev "$ifname"
}

connect_host() {
    local req_ip=$1
    local ifname="veth-bb-host"
    local peername="veth-bb-gw"

    if ! docker network inspect "$NETWORK_NAME" >/dev/null 2>&1; then
        echo "Error: Network '$NETWORK_NAME' does not exist. Run '$0 create-network' first."
        exit 1
    fi

    # Determine image to use for host gateway container
    local image="$IMAGE_NAME_COMMON"
    if ! docker image inspect "$image" >/dev/null 2>&1; then
        if docker image inspect "$IMAGE_NAME_TARGET" >/dev/null 2>&1; then
            image="$IMAGE_NAME_TARGET"
        elif docker image inspect "$IMAGE_NAME_PROJECT" >/dev/null 2>&1; then
            image="$IMAGE_NAME_PROJECT"
        elif docker image inspect "ubuntu:24.04" >/dev/null 2>&1; then
            image="ubuntu:24.04"
        else
            echo "Building common image '$IMAGE_NAME_COMMON' first..."
            build_common
        fi
    fi

    # Clean up existing host gateway container if present
    docker rm -f bitbots-host-gateway >/dev/null 2>&1 || true

    local ip_args=()
    if [ -n "$req_ip" ]; then
        local clean_ip="${req_ip%%/*}"
        ip_args=(--ip "$clean_ip")
    else
        ip_args=(--ip "10.66.0.254")
    fi

    echo "Starting host gateway container on '$NETWORK_NAME'..."
    local started=0
    if [ ${#ip_args[@]} -gt 0 ]; then
        if docker run -d \
            --name bitbots-host-gateway \
            --network "$NETWORK_NAME" \
            "${ip_args[@]}" \
            --cap-add=NET_ADMIN \
            --sysctl net.ipv4.ip_forward=1 \
            --restart unless-stopped \
            "$image" \
            sleep infinity >/dev/null 2>&1; then
            started=1
        else
            echo "Note: Could not assign ${ip_args[*]}. Falling back to dynamic IP allocation..."
        fi
    fi

    if [ "$started" -eq 0 ]; then
        docker run -d \
            --name bitbots-host-gateway \
            --network "$NETWORK_NAME" \
            --cap-add=NET_ADMIN \
            --sysctl net.ipv4.ip_forward=1 \
            --restart unless-stopped \
            "$image" \
            sleep infinity >/dev/null
    fi

    local cpid
    cpid=$(docker inspect -f '{{.State.Pid}}' bitbots-host-gateway)
    local assigned_ip
    assigned_ip=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' bitbots-host-gateway)

    if [ -z "$cpid" ] || [ "$cpid" -eq 0 ]; then
        echo "Error: Failed to get process ID for bitbots-host-gateway container."
        exit 1
    fi

    if [ "$(id -u)" -ne 0 ]; then
        echo "Configuring host routing to Docker overlay network requires root privileges (sudo)."
        sudo bash -c "$(declare -f _do_connect_host); _do_connect_host '$cpid' '$ifname' '$peername'"
    else
        _do_connect_host "$cpid" "$ifname" "$peername"
    fi

    echo "Successfully connected host to '$NETWORK_NAME' (gateway IP: $assigned_ip on $ifname)."
    echo "The host PC is now directly part of the multi-PC network."
    echo "You can now directly reach all containers on this PC and remote PCs (e.g., 'ssh bitbots@10.66.6.2' or 'pixi run deploy mickey')."
}

disconnect_host() {
    local ifname="veth-bb-host"
    if docker ps -a --format "{{.Names}}" | grep -q "^bitbots-host-gateway$"; then
        echo "Stopping host gateway container..."
        docker stop "bitbots-host-gateway" >/dev/null 2>&1 || true
        docker rm "bitbots-host-gateway" >/dev/null 2>&1 || true
    fi
    if ip link show "$ifname" >/dev/null 2>&1; then
        echo "Disconnecting host interface $ifname..."
        if [ "$(id -u)" -ne 0 ]; then
            sudo ip link delete "$ifname" 2>/dev/null || true
        else
            ip link delete "$ifname" 2>/dev/null || true
        fi
        echo "Host disconnected from container network."
    else
        echo "Host interface $ifname is not connected."
    fi
}

net_shell() {
    if ! command -v docker >/dev/null; then
        echo "Error: docker not found."
        exit 1
    fi

    if docker ps --format "{{.Names}}" | grep -q "^bitbots-host-gateway$"; then
        echo "Entering Bit-Bots network shell via host gateway..."
        echo "Inside this shell, all containers across all PCs on '$NETWORK_NAME' are directly reachable."
        echo "Type 'exit' to return to your normal shell."
        docker exec -it bitbots-host-gateway bash
        return
    fi

    local cid
    cid=$(docker ps --filter "network=$NETWORK_NAME" -q | head -n 1)
    if [ -n "$cid" ]; then
        local cname
        cname=$(docker inspect -f '{{.Name}}' "$cid" | sed 's/^\///')
        echo "Entering Bit-Bots network shell via container '$cname'..."
        echo "Inside this shell, all containers across all PCs on '$NETWORK_NAME' are directly reachable."
        echo "Type 'exit' to return to your normal shell."
        docker exec -it "$cid" bash
        return
    fi

    local netns_file
    netns_file=$(find_overlay_netns || true)
    if [ -n "$netns_file" ]; then
        echo "Entering Docker Swarm overlay network namespace ($netns_file)..."
        echo "Inside this shell, all containers on '$NETWORK_NAME' are directly reachable via IP."
        echo "Type 'exit' to return to your normal shell."
        if [ "$(id -u)" -eq 0 ]; then
            nsenter --net="$netns_file" bash
        else
            sudo nsenter --net="$netns_file" bash
        fi
        return
    fi

    echo "---------------------------------------------------------"
    echo "  BIT-BOTS NETWORK SHELL"
    echo "---------------------------------------------------------"
    echo "No running container found on '$NETWORK_NAME'."
    echo "Start a container or connect the host first:"
    echo "  - Connect host: $0 connect-host"
    echo "  - Run target:   $0 run-target <name>"
    echo "---------------------------------------------------------"
    export BITBOTS_NET_SHELL=1
    ${SHELL:-bash}
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
        # Direct connection if in net-shell, running as root, or if IP is directly reachable (e.g. via connect-host or bridge)
        ssh "${ssh_opts[@]}" "bitbots@$ip"
    else
        # Try to find container ID for proxying in case IP is not directly routed
        local ip_slug=${ip//./-}
        local cid=$(docker ps --format "{{.ID}} {{.Names}}" | grep "bitbots" | grep "$ip_slug" | head -n 1 | cut -d' ' -f1)
        if [ -z "$cid" ]; then
            # If target container is on another Swarm node, proxy through any local container on the overlay network
            cid=$(docker ps --filter "network=$NETWORK_NAME" -q | head -n 1)
        fi
        if [ -n "$cid" ]; then
            ssh "${ssh_opts[@]}" \
                -o "ProxyCommand=docker exec -i $cid nc $ip 22" \
                "bitbots@$ip"
        else
            echo "Error: Container with IP $ip not reachable and no local proxy container found on '$NETWORK_NAME'."
            echo "Options to enable connectivity:"
            echo "  1. Run '$0 connect-host' with sudo to connect your host directly to the overlay network."
            echo "  2. Run '$0 net-shell' to open a shell inside the overlay network."
            echo "  3. Start a container on this machine to enable proxying."
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
    swarm-init)
        swarm_init "$2"
        ;;
    swarm-join)
        shift
        swarm_join "$@"
        ;;
    swarm-leave)
        swarm_leave
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
    connect-host)
        connect_host "$2"
        ;;
    disconnect-host)
        disconnect_host
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
