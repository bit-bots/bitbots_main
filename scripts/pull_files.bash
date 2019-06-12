#!/bin/bash

META=$(dirname $(dirname $(realpath $0)))

mkdir -p $META/bitbots_vision/bitbots_vision/models/classifier_01
wget -r -N -np -nd -P $META/bitbots_vision/bitbots_vision/models/classifier_01 --reject "index.html*" "http://data.bit-bots.de/models/classifier_01/"

mkdir -p $META/bitbots_vision/bitbots_vision/models/fcnn03
wget -r -N -np -nd -P $META/bitbots_vision/bitbots_vision/models/fcnn03 --reject "index.html*" "http://data.bit-bots.de/models/fcnn03/"

mkdir -p $META/bitbots_vision/bitbots_vision/models/fcnn031
wget -r -N -np -nd -P $META/bitbots_vision/bitbots_vision/models/fcnn031 --reject "index.html*" "http://data.bit-bots.de/models/fcnn031/"

mkdir -p $META/bitbots_vision/bitbots_vision/models/fcnn032
wget -r -N -np -nd -P $META/bitbots_vision/bitbots_vision/models/fcnn032 --reject "index.html*" "http://data.bit-bots.de/models/fcnn032/"
