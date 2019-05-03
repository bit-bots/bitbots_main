#!/bin/bash

git submodule foreach git status --short | sed "s/Entering '\(.*\)'$/\1:/"
