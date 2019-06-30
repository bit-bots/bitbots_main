#!/bin/bash
set -e

git submodule update --init lib/dwa_local_planner
git -C lib/dwa_local_planner checkout master
git submodule update --init lib/vision_opencv
git -C lib/vision_opencv checkout melodic
git submodule foreach git pull
if [[ -d basler_drivers ]]; then
    git -C basler_drivers pull
else
    echo -e "\n\n\033[91m\033[1mNo basler drivers found! Use 'make basler' to install them and restart this script afterwards\033[0m\n"
    exit 1
fi
