#!/bin/sh

# Install pre-commit hooks for all submodules that have a .pre-commit-config.yaml file
git submodule foreach "test -f .pre-commit-config.yaml && pre-commit install || :"
