#!/bin/bash
set -e

git submodule foreach git pull
git -C humanoid_league_misc submodule update
