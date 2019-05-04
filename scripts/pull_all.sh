#!/bin/sh
git submodule update --init lib/dwa_local_planner
git -C lib/dwa_local_planner checkout master
git submodule foreach git pull
