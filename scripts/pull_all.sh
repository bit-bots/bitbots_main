#!/bin/sh

{
    git submodule update --recursive --remote --init &&
    git submodule foreach -q --recursive 'branch="$(git config -f $toplevel/.gitmodules submodule.$name.branch)"; [ "$branch" = "" ] && git checkout master || git checkout $branch' &&
    git submodule sync &&
    git submodule foreach git pull
} || echo -e "\033[91m\033[1m########## Pull failed! ##########\033[0m"
