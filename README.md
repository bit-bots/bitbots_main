# Bit-Bots Software Stack

[![Build & Test](https://github.com/bit-bots/bitbots_main/actions/workflows/ci.yml/badge.svg)](https://github.com/bit-bots/bitbots_main/actions/workflows/ci.yml)
[![Code style checks](https://github.com/bit-bots/bitbots_main/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/bit-bots/bitbots_main/actions/workflows/pre-commit.yml)
[![ROS Version Jazzy](https://img.shields.io/badge/ROS%20Version-Jazzy-00b8ff)](https://docs.ros.org/en/jazzy/index.html)

This git repository contains all RoboCup-related code and documentation from the Hamburg Bit-Bots team.
All code is written as individual ROS 2 packages targeting Ubuntu.

<p align="center">
  <img width="30%" src="logo.png" alt="Bit-Bots Logo" />
</p>

## Installation

Full step-by-step instructions for installing the Bit-Bots software stack and ROS 2 can be found in our documentation [here](https://doku.bit-bots.de/meta/manual/tutorials/install_software_ros2.html).


Run the following command to install all dependencies and clone all necessary repositories. Make sure you have [just](https://just.systems/) and correct ROS 2 version installed.

``` shell
just install
```

## Building

To build the code, run

``` shell
just build
```

and source the setup file with

``` shell
. install/setup.sh
```

To see all available recipes, run

``` shell
just -l
```

## Run auto formatting

To format all code in the repository, run

``` shell
just format
```

## More documentation

Our documentation is hosted on [docs.bit-bots.de](https://docs.bit-bots.de/).
