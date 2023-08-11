# bitbots_meta

This git repository contains all RoboCup-related code and documentation from the Hamburg Bit-Bots team as git submodules.
All code is written as ROS 2 packages for ROS 2 on Ubuntu.

[![Test if all packages build](https://github.com/bit-bots/bitbots_meta/actions/workflows/build.yml/badge.svg)](https://github.com/bit-bots/bitbots_meta/actions/workflows/build.yml)

## Installation

Full step-by-step instructions for installing the Bit-Bots software stack and ROS 2 can be found in our documentation [here](https://doku.bit-bots.de/meta/manual/tutorials/install_software_ros2.html).

To download the Bit-Bots software stack clone this repository.

```bash
git clone git@github.com:bit-bots/bitbots_meta.git
```

The meta repository mainly contains references (submodules) to other repositories for the different components of the codebase.
Download them run the following commands.

```bash
cd bitbots_meta
make pull-init
```

## Update the codebase

If you want to update all submodules, this repo, and supplementing files, run

``` bash
make pull-all
```

Make sure you are on the correct branch and have no uncommitted changes in each submodule.
To check this run

```bash
make status
```

Other scripts are available in the `scripts` folder, [documented here](scripts/README.md).

## Repository Structure

The naming prefix of submodules indicates the scope of the packages.

* bitbots_ : specific RoboCup code of our team which follows interface specification of humanoid_league_msgs
* humanoid_league_ : packages which are useful for all teams in the RoboCup Humanoid League, e.g. visualization tools and game controller
* no prefix : packages which are useful in general and usable outside of RoboCup
* lib : folder for third party libraries that need to be build from source

## Documentation

Our documentation is hosted on [docs.bit-bots.de](https://docs.bit-bots.de/).
