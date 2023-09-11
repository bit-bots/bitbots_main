#!/bin/bash

META=$(dirname $(dirname $(realpath $0)))
wget --no-verbose --show-progress --recursive --timestamping --no-parent --no-host-directories --directory-prefix=$META/bitbots_vision/bitbots_vision --reject "index.html*" "https://data.bit-bots.de/models/"

wget --no-verbose --show-progress --recursive --timestamping --no-parent --no-host-directories --directory-prefix=$META/bitbots_motion/bitbots_rl_motion --reject "index.html*" "https://data.bit-bots.de/rl_walk_models/"
