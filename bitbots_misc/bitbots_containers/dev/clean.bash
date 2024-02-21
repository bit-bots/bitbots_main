#!/usr/bin/env bash

IMAGE_NAME=bitbots-dev-iron
CONTAINER_NAME=bitbots-dev-iron

# Ask for confirmation
if [[ -n $CONTAINER_NAME ]]; then
    read -p "Do you really want delete the container? You will need to reinstall all deps and loose all changes outside the bitbots_main repo! [y/N] " -n 1 -r
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Stop and remove the container
docker stop $CONTAINER_NAME > /dev/null  # Do not print container name
docker rm $CONTAINER_NAME > /dev/null  # Do not print container name
