#!/bin/bash

META=$(dirname $(dirname $(realpath $0)))

wget -r -N -np -nH -P $META/bitbots_vision/bitbots_vision --reject "index.html*" "http://data.bit-bots.de/models/"
