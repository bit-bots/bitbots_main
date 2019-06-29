#!/bin/bash

META=$(dirname $(dirname $(realpath $0)))
rm -rf $META/bitbots_vision/bitbots_vision/models/fcnn03 $META/bitbots_vision/bitbots_vision/models/fcnn031 $META/bitbots_vision/bitbots_vision/models/fcnn032 $META/bitbots_vision/bitbots_vision/models/classifier01
wget -r -N -np -nH -P $META/bitbots_vision/bitbots_vision --reject "index.html*" "http://data.bit-bots.de/models/"
