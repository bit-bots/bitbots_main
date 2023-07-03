# bitbots_meta
This git repository contains all RoboCup-related code and documentation from the Hamburg Bit-Bots team as git submodules.
All code is written as ROS 2 packages for ROS 2 on Ubuntu.

[![Test if all packages build](https://github.com/bit-bots/bitbots_meta/actions/workflows/build.yml/badge.svg)](https://github.com/bit-bots/bitbots_meta/actions/workflows/build.yml)

## Installation

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

If you want to pull all submodules run

``` bash
make pull-all
```

This will pull all reposiories. Make sure you are on the correct branch and have no uncommited changes in each submodule.
To check this run

```bash
make status
```

## Structure

The naming prefix indicates the scope of the packages.

 * bitbots_ : specific RoboCup code of our team which follows interface specification of humanoid_league_msgs
 * humanoid_league_ : packages which are useful for all teams in the RoboCup Humanoid League, e.g. visualization tools and gamecontroller
 * no prefix : packages which are useful in general and usable outside of RoboCup
 * lib : folder for third party libraries that need to be build from source

## Documentation

Our documentation is WIP but what is already available is hosted on [doku.bit-bots.de](http://doku.bit-bots.de/meta/)

You can also build the documentation yourself
``` bash
make doc
```

and open it with

``` bash
firefox doc/html/index.html
```

You can also generate an overview of the software using the script in the architecture folder.

