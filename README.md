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

Make sure you are on the correct branch and have no uncommited changes in each submodule.
To check this run

```bash
make status
```

Other scripts are available in the `scripts` folder, [documented here](scripts/README.md).

## Repository Structure

The naming prefix of submodules indicates the scope of the packages.

* bitbots_ : specific RoboCup code of our team which follows interface specification of humanoid_league_msgs
* humanoid_league_ : packages which are useful for all teams in the RoboCup Humanoid League, e.g. visualization tools and gamecontroller
* no prefix : packages which are useful in general and usable outside of RoboCup
* lib : folder for third party libraries that need to be build from source

## Recommended VSCode extensions

```bash
code --install-extension cschlosser.doxdocgen && \  # Generates doxygen comments
code --install-extension DavidAnson.vscode-markdownlint && \  # Lints markdown files
code --install-extension DotJoshJohnson.xml && \  # XML support
code --install-extension eamodio.gitlens && \  # Extended git highlighting
code --install-extension mohsen1.prettify-json && \  # JSON formatting
code --install-extension ms-azuretools.vscode-docker && \  # Docker support
code --install-extension MS-CEINTL.vscode-language-pack-de && \  # German language pack
code --install-extension ms-iot.vscode-ros && \  # ROS support
code --install-extension ms-python.python && \  # Python support
code --install-extension ms-python.vscode-pylance && \  # Python support
code --install-extension ms-vscode.cmake-tools && \  # CMake support
code --install-extension ms-vscode.cpptools && \  # C++ support
code --install-extension ms-vscode.cpptools-extension-pack && \  # C++ support
code --install-extension ms-vscode.cpptools-themes && \  # C++ support
code --install-extension njpwerner.autodocstring && \  # Generates docstrings
code --install-extension streetsidesoftware.code-spell-checker && \  # Spell checker
code --install-extension streetsidesoftware.code-spell-checker-german && \  # Spell checker
code --install-extension tamasfe.even-better-toml && \  # TOML support
code --install-extension twxs.cmake && \  # CMake support
code --install-extension zxh404.vscode-proto3  # Protobuf support
```

## Documentation

Our documentation is hosted on [docs.bit-bots.de](https://docs.bit-bots.de/).
