#!/usr/bin/env bash
set -eEou pipefail

IMAGE_NAME=bitbots-dev-iron
CONTAINER_NAME=`docker ps --filter status=running --filter ancestor=$IMAGE_NAME --format "{{.Names}}"`

# Enable xhost access
xhost +local:root

if [[ -z $CONTAINER_NAME ]]; then
    echo "Container not running, starting it"
    DIR=`dirname $0`
    bash -c "$DIR/start.bash"
fi

CONTAINER_NAME=`docker ps --filter status=running --filter ancestor=$IMAGE_NAME --format "{{.Names}}"`

if [[ -z $@ ]]; then
	docker exec -it $CONTAINER_NAME /usr/bin/zsh
else
	docker exec -it $CONTAINER_NAME "$@"
fi
