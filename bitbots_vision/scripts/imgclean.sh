#!/usr/bin/env bash

# This is a small bash script, to quickly sort a directory of images
# using feh with shortcuts.
# Start this inside the directory of images to sort.
#
# Press key:
# 1 -> move current image to $Trash subdirectory
# 2 -> Move current image to $Balls subdirectory
# 3 -> Move current image to $Goals subdirectory
# use right/left arrow keys to navigate through images and keep the current image inside its original directory

Trash="unusable"
Balls="balls"
Goals="goalposts"

mkdir -p $Trash $Balls $Goals

feh -Z -F -d --action1 "mv '%f' $Trash" --action2 "mv '%f' $Balls" --action3 "mv '%f' $Goals"
