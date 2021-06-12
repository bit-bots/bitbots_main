#!/bin/bash
set -e

git submodule foreach git pull
git -C humanoid_league_misc git submodule update
if [[ -d basler_drivers ]]; then
    git -C basler_drivers pull
else
    echo -e "\n\n\033[91m\033[1mNo basler drivers found! Use 'make basler' to install them and restart this script afterwards\033[0m\n"
fi
