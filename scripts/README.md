# bitbots_meta scripts

This directory contains scripts that are very useful for development, testing and deployment.

## `build-doc.py`

To build documentation with `sphinx` for all packages in the workspace. Possibly outdated.

## `catkin-to-ament.py`

To help migration from a ROS 1 package to ROS2.

## `deploy_robots.py`

Deploy, configure, and launch the Bit-Bots software remotely on a robot.
This tool can target all, multiple, or single robots at once, specified by their hostname, robot name, or IP address.
Five different tasks can be performed:

1. Synchronize the local source code to the target workspace
2. Install ROS 2 dependencies using `rosdep` on the target
3. Configure game-settings and wifi on the target
4. Build (compile) the workspace on the target
5. Launch the teamplayer software on the target

### Example usage

- Get help and list all arguments:

    ```bash
    ./deploy_robots.py --help
    ```

- Default usage: Synchronize, install dependencies,  and build on all robots:

    ```bash
    ./deploy_robots.py <target>
    ```

- Also launch the teamplayer software on all robots:

    ```bash
    ./deploy_robots.py --launch-teamplayer ALL
    ```

TODO: Update this example

- Only build the `bitbots_utils` ROS package on the `nuc1` host:

    ```bash
    ./deploy_robots.py --build-only --package bitbots_utils nuc1
    ```

## `gen_rdmanifest.py`

Generate .rdmanifest files for a package. These files are necessary to describe how rosdep can install a package via the 'source' package manager

## `git_status.bash`

Print the git status of all submodules.
Can be invoked by `make status` in the root directory.

## `install.pl`

To create a workspace with all dependencies installed.
Unfortunatly, this script is vastly outdated.

## `pull_all.sh`

To pull all submodules and the meta repository.
Can be invoked by `make pull-all` in the root directory.

## `pull_files.sh`

To pull all supplemental files (like ML models).
Can be invoked by `make pull-files` in the root directory.
Also included with `make pull-all`.

## `pull_init.sh`

To initialize, clone and pull all submodules.
Neccessary in a fresh workspace.
Can be invoked by `make pull-init` in the root directory.
