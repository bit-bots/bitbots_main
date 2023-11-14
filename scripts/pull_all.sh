#!/bin/bash
set -e

# This is a temporary hack, so it is not necessary that every user removes these dirs themselfs
rm -rf basler_drivers
rm -rf humanoid_league_msgs

git submodule foreach git pull
git -C humanoid_league_misc submodule update
