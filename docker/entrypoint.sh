#!/bin/bash
set -e

# Start SSH daemon in background
/usr/sbin/sshd

# Export ROS_DOMAIN_ID to /etc/environment for SSH sessions if provided
if [ -n "$ROS_DOMAIN_ID" ]; then
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> /etc/environment
fi

# If custom command was passed (and is not default CMD or sshd), execute it
if [ "$#" -gt 0 ] && [ "$1" != "default" ] && [ "$1" != "/usr/sbin/sshd" ] && [ "$1" != "/usr/sbin/sshd -D" ]; then
    exec "$@"
fi

# Run Zenoh router if pixi workspace is available
if [ -f "/home/bitbots/bitbots_main/pixi.toml" ]; then
    if [ "$SIMULATOR" = "1" ] || [ "$ZENOH_MODE" = "router" ]; then
        echo "Starting Zenoh router (mode=router)..."
        exec su - bitbots -c "cd /home/bitbots/bitbots_main && /home/bitbots/.pixi/bin/pixi run -e default ros2 run rmw_zenoh_cpp rmw_zenohd"
    else
        SIMULATOR_ENDPOINT="${SIMULATOR_ENDPOINT:-tcp/simulator:7447}"
        echo "Starting Zenoh router (mode=peer, target=$SIMULATOR_ENDPOINT)..."
        exec su - bitbots -c "cd /home/bitbots/bitbots_main && ZENOH_CONFIG_OVERRIDE='mode=\"peer\";connect/endpoints=[\"${SIMULATOR_ENDPOINT}\"]' /home/bitbots/.pixi/bin/pixi run -e default ros2 run rmw_zenoh_cpp rmw_zenohd"
    fi
else
    echo "Workspace /home/bitbots/bitbots_main/pixi.toml not found; keeping container alive..."
    exec tail -f /dev/null
fi
