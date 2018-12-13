#!/bin/sh



git submodule update --recursive --remote --init




git submodule foreach -q --recursive 'branch="$(git config -f $toplevel/.gitmodules submodule.$name.branch)"; [ "$branch" = "" ] && git checkout master || git checkout $branch'

#git submodule foreach --recursive  'git checkout $(git rev-parse --abbrev-ref HEAD || echo master)'
git submodule sync

git submodule foreach  git pull

git submodule status --recursive
