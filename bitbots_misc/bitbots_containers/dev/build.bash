#!/usr/bin/env bash
set -eEou pipefail

IMAGE_NAME=bitbots-dev-iron

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# Build the docker image
docker build \
  --pull \
  --network=host \
  --build-arg user=$USER \
  --build-arg uid=$UID \
  --build-arg home=$HOME \
  -t $IMAGE_NAME \
  $@ \
  $SCRIPTPATH
