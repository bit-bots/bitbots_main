#!/bin/sh

# Update rosdep and install dependencies from meta directory
rosdep update
rosdep install -iry --from-paths .
