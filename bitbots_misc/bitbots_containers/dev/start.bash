#!/usr/bin/env bash
set -eEou pipefail

IMAGE_NAME=bitbots-dev-iron
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# Test whether a running container exists
CONTAINER_NAME=`docker ps --filter status=running --filter ancestor=$IMAGE_NAME --format "{{.Names}}"`
if [[ -n $CONTAINER_NAME ]]; then
    echo "Container already running, connect with ./connect or stop it with ./stop"
    exit 1
fi

# Test whether a stopped container exists
CONTAINER_NAME=`docker ps --filter status=exited --filter ancestor=$IMAGE_NAME --format "{{.Names}}"`
if [[ -n $CONTAINER_NAME ]]; then
    echo "Resuming stopped container"
    docker start $CONTAINER_NAME > /dev/null
else
    echo "Starting new container"
    echo "You will need to setup the bitbots main repo in the container"
    echo "This means you will need to run the following commands in the container:"
    echo "'make -C ~/colcon_ws/src/bitbots_main install'"
    # Run the container with shared X11
    docker run -d \
      --net=host \
      --add-host=$HOSTNAME:127.0.1.1 \
      -e DISPLAY \
      -e DOCKER=1 \
      -v "$HOME:/srv/host_home:rw" \
      -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      -v "$HOME/.ssh:$HOME/.ssh:rw" \
      -v "$(realpath $SCRIPTPATH/../../../):$HOME/colcon_ws/src/bitbots_main:rw" \
      -e LIBGL_ALWAYS_SOFTWARE=1 \
      --ulimit nofile=1024:524288 \
      --device=/dev/dri:/dev/dri \
      --cap-add=SYS_PTRACE \
      --security-opt seccomp=unconfined \
      --name "bitbots-dev-iron" \
      -it $IMAGE_NAME > /dev/null  # Do not print container id
fi
