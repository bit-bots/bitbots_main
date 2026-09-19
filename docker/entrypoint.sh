#!/bin/bash
set -e

# Start SSH daemon in background
/usr/sbin/sshd

# Export ROS_DOMAIN_ID to /etc/environment for SSH sessions if provided
if [ -n "$ROS_DOMAIN_ID" ]; then
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> /etc/environment
fi

# Export Zenoh configuration URIs to /etc/environment for SSH sessions
ZENOH_SESSION_CONFIG_URI="${ZENOH_SESSION_CONFIG_URI:-/home/bitbots/.config/zenoh/session.json5}"
ZENOH_ROUTER_CONFIG_URI="${ZENOH_ROUTER_CONFIG_URI:-/home/bitbots/.config/zenoh/router.json5}"
export ZENOH_SESSION_CONFIG_URI
export ZENOH_ROUTER_CONFIG_URI
echo "ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI" >> /etc/environment
echo "ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI" >> /etc/environment

# Adjust router config target endpoint if simulator IP / endpoint is configured
if [ -f "$ZENOH_ROUTER_CONFIG_URI" ]; then
    SIM_TARGET="${SIMULATOR_IP:-$SIMULATOR_ENDPOINT}"
    if [ -n "$SIM_TARGET" ] && [ "$SIM_TARGET" != "none" ] && [ "$SIMULATOR" != "1" ]; then
        if [[ "$SIM_TARGET" != tcp/* ]]; then
            if [[ "$SIM_TARGET" != *:* ]]; then
                SIM_TARGET="tcp/${SIM_TARGET}:7447"
            else
                SIM_TARGET="tcp/${SIM_TARGET}"
            fi
        fi
        sed -i "s|// __SIMULATOR_ENDPOINT__|      \"${SIM_TARGET}\",|" "$ZENOH_ROUTER_CONFIG_URI"
    fi
fi

# If custom command was passed (and is not default CMD or sshd), execute it
if [ "$#" -gt 0 ] && [ "$1" != "default" ] && [ "$1" != "/usr/sbin/sshd" ] && [ "$1" != "/usr/sbin/sshd -D" ]; then
    exec "$@"
fi

# Run Zenoh router only if explicitly requested
if [ "$START_ZENOH_ROUTER" = "1" ] || [ "$START_ZENOH_ROUTER" = "true" ] || [ "$ZENOH_ROUTER" = "1" ] || [ "$ZENOH_ROUTER" = "true" ]; then
    if [ -f "/home/bitbots/bitbots_main/pixi.toml" ]; then
        echo "Starting Zenoh router..."
        exec su - bitbots -c "cd /home/bitbots/bitbots_main && /home/bitbots/.pixi/bin/pixi run -e default ros2 run rmw_zenoh_cpp rmw_zenohd"
    else
        echo "Workspace /home/bitbots/bitbots_main/pixi.toml not found; keeping container alive..."
    fi
fi

# Keep container alive if no foreground process is running
exec tail -f /dev/null
