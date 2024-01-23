# Scripts

This directory contains scripts that are very useful for development, testing and deployment.

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

- Default usage: Run all tasks on the `nuc1` host:

    ```bash
    ./deploy_robots.py nuc1
    ```

- Make all robots ready for games. This also launch the teamplayer software on all robots:

    ```bash
    ./deploy_robots.py ALL
    ```

- Only run the sync and build tasks on the `nuc1` and `nuc2` hosts:

    ```bash
    ./deploy_robots.py --sync --build nuc1 nuc2
    ```

- Only build the `bitbots_utils` ROS package on the `nuc1` host:

    ```bash
    ./deploy_robots.py --package bitbots_utils nuc1
    ```

## `make_basler.sh`

Downloads and installs the Basler Pylon SDK for Linux. This is needed to communicate with our camera. Normally called by the `make basler` or other `make` targets.
