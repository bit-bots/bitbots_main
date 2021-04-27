#!/bin/bash
set -e
set -x

source $BITBOTS_CATKIN_WORKSPACE/devel/setup.bash

exec "$@"