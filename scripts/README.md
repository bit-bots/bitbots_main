# Scripts

This directory contains scripts that are very useful for development, testing and deployment.

## `deploy_robots.py`

Deploy, configure, and launch the Bit-Bots software remotely on robots.
It is also callable via `pixi run deploy <arguments>`.

These tasks can be performed, by default all of them in this order:

1. Synchronize the local source code to the target workspace
2. Configure game settings and wifi on the target
3. Build (compile) the workspace on the target
4. Launch the teamplayer software on the target

### Targets

One or more space-separated targets can be given. A target can be:

- a robot's number,
- a robot's name or one of its aliases (case-insensitive),
- an IP address, either of a known robot or of any other machine,
- `ALL` to target all known robots.

Known robots are defined in [`deploy/known_targets.yaml`](deploy/known_targets.yaml), each with a name, a number, optional aliases and its IP addresses.
All IP addresses of a robot are tried in parallel and the first reachable one is used, so the same target works in the lab and on the field.
Each robot is only deployed to once, even if it is given multiple times, e.g. by name and by number.
Run `./deploy_robots.py --show-targets` to list all known robots.

To add a robot, append an entry to `known_targets.yaml`. The file is validated on startup, and invalid or ambiguous entries, such as a name or IP address used by two robots, are reported as errors.

### Example usage

- Get help and list all arguments:

    ```shell
    ./deploy_robots.py --help
    ```

- Default usage: Run all tasks on Kalliope:

    ```shell
    ./deploy_robots.py kalliope
    ```

- Make all robots ready for games. This also launches the teamplayer software on all robots:

    ```shell
    ./deploy_robots.py ALL
    ```

- Only run the sync and build tasks on two robots, given by number and by name:

    ```shell
    ./deploy_robots.py --sync --build 1 romeo
    ```

- Only build the `bitbots_utils` ROS package on Kalliope:

    ```shell
    ./deploy_robots.py --package bitbots_utils kalliope
    ```
