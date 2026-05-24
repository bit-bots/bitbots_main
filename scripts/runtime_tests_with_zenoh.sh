#!/usr/bin/env bash
set -eEuo pipefail

# This script is used to run the ROS 2 tests with the Zenoh RMW implementation.
# It starts the Zenoh daemon in the background and ensures it is properly cleaned up after the tests are done.


# Store the PID of the Zenoh daemon to ensure we can clean it up properly
zenoh_pid=""

cleanup() {
    # If the Zenoh daemon is still running, kill it
    if [[ -n "$zenoh_pid" ]]; then
        kill "$zenoh_pid" 2>/dev/null || true
        wait "$zenoh_pid" 2>/dev/null || true
        echo "Cleaned up Zenoh daemon with PID $zenoh_pid"
    fi
}

# Set up traps to ensure cleanup is called on script exit, interrupt, or termination
trap cleanup EXIT INT TERM

# Start the Zenoh daemon in the background and store its PID
ros2 run rmw_zenoh_cpp rmw_zenohd &
zenoh_pid=$!

echo "Started Zenoh daemon with PID $zenoh_pid"

# Run the colcon tests with the provided arguments ($@ allows passing additional arguments to colcon test)
colcon test "$@"
