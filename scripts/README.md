# Scripts

This directory contains scripts that are very useful for development, testing and deployment.

This tool is also callable via `pixi run deploy <arguments>`.

## `deploy_robots.py`

Deploy, configure, and launch the Bit-Bots software remotely on a robot.
This tool can target all, multiple, or single robots at once, specified by their hostname, robot name, or IP address.
These different tasks can be performed:

1. Synchronize the local source code to the target workspace
2. Configure game-settings and wifi on the target
3. Build (compile) the workspace on the target
4. Launch the teamplayer software on the target

### Example usage

- Get help and list all arguments:

    ```shell
    ./deploy_robots.py --help
    ```

- Default usage: Run all tasks on the `nuc1` host:

    ```shell
    ./deploy_robots.py nuc1
    ```

- Make all robots ready for games. This also launch the teamplayer software on all robots:

    ```shell
    ./deploy_robots.py ALL
    ```

- Only run the sync and build tasks on the `nuc1` and `nuc2` hosts:

    ```shell
    ./deploy_robots.py --sync --build nuc1 nuc2
    ```

- Only build the `bitbots_utils` ROS package on the `nuc1` host:

    ```shell
    ./deploy_robots.py --package bitbots_utils nuc1
    ```

## `deploy_game.py`

Deploy and orchestrate a multi-robot simulation game setup across multiple host machines.
This script takes a configuration file specifying the game setup (e.g. `4:4`, `2:2`) and available hosts, assigns robot roles and domain IDs (allocating 2 fewer robots to the simulator host for equal load distribution), triggers deployment on the hosts, and outputs the commands to start host UDP bridge configuration containers (`udp_via_zenoh` with `config_sim.toml` / `config_robot.toml`), robot containers with Zenoh, and launch teamplayer instances.

### Example usage

- Deploy a game using a configuration file:

    ```shell
    ./deploy_game.py game_config.yaml
    ```

- Display the deployment plan and required host commands without executing the deploy step:

    ```shell
    ./deploy_game.py game_config.yaml --dry-run
    ```
