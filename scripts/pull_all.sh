#!/bin/sh
git submodule update --init lib/dwa_local_planner
git submodule foreach git pull
