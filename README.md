# Bit-Bots Software Stack

[![Build & Test](https://github.com/bit-bots/bitbots_main/actions/workflows/ci.yml/badge.svg)](https://github.com/bit-bots/bitbots_main/actions/workflows/ci.yml)
[![Code style checks](https://github.com/bit-bots/bitbots_main/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/bit-bots/bitbots_main/actions/workflows/pre-commit.yml)
[![ROS Version Jazzy](https://img.shields.io/badge/ROS%20Version-Jazzy-00b8ff)](https://docs.ros.org/en/jazzy/index.html)

This git repository contains all RoboCup-related code and documentation from the Hamburg Bit-Bots team.
All code is written as individual ROS 2 packages.

<p align="center">
  <img width="30%" src="logo.png" alt="Bit-Bots Logo" />
</p>

## Installation

The workspace is managed using the [pixi](https://pixi.sh) package manager. This allows us to have reproducible builds and easy user space dependency management similar to tools like `uv` or `cargo`. This also means that no system wide ROS installation or superuser privileges are required to install or run the code.

Follow our [installation guide](https://docs.bit-bots.de/meta/manual/tutorials/installation.html) for a quick installation or step-by-step instructions.

## Using the workspace

To activate the workspace run the following command in the terminal you want to use:

``` shell
pixi shell
```

alternatively, you can run individual commands inside the workspace without activating the shell:

``` shell
pixi run <command>
```

To build the workspace, run the following command in the terminal:

``` shell
pixi run build
```

To see some predefined tasks / commands, run

``` shell
pixi task list
```

## Deploy to robots

To deploy the software to a robot, run

``` shell
pixi run deploy
```

For more information on the deployment tooling, see [this documentation](scripts/README.md).

## Run auto formatting

To format all code in the repository, run

``` shell
pixi run format
```

## More documentation

Our documentation is hosted on [docs.bit-bots.de](https://docs.bit-bots.de/).
