# Configure & Launch a Robot

This section describes how to configure and deploy your current software to a robot for game preparation.

!!! note

    See `starting_robot` for the prerequisites and safety warnings on when it is safe to start the robot and how the software is synced and built.

!!! note

    See `robots` for the robot names and IP addresses, and `setup_robot` for the one-time Ansible provisioning of a robot.

## Deployment

Deployment is the process of preparing a robot for the next game and starting the correct software.

At a competition, follow these steps:

1.  **Configure Wi-Fi networks for fields:** This needs to be done before the competition at the team area (see `competition_wifi`)!

2.  **Checkout the latest code:** In your local [bitbots_main](https://github.com/bit-bots/bitbots_main) repo run:

    1.  Check that you are on the `main` branch
    2.  `git pull` to get the latest changes

3.  **Sync, configure, compile and launch software:** In the `bitbots_main` directory run the deploy tool:

    ``` bash
    pixi run deploy <robot_name | ALL>
    ```

    This does the following tasks:

    - Synchronize/Copy the current state of your local bitbots_main directory to the robot(s)
    - Install necessary dependencies on the robot(s)
    - Configure game specific settings and the Wi-Fi connection on the robot(s)
    - Build/Compile the source code you just synchronized to the robot(s)
    - Launch the teamplayer software on the robot(s)

    If you need help with this tool, or want other options, look at [this README](https://github.com/bit-bots/bitbots_main/blob/main/scripts/README.md#deploy_robots-py) for example usages or call:

    ``` bash
    pixi run deploy -h
    ```

4.  **Optional: Connect to the robot:** Simply copy-paste the command provided by the deploy-tool when its finished. See `connecting` for how to connect and how to use tmux.

5.  **Profit!** The robot is now ready play!

## Cleanup

!!! note "TODO"

    Rework of the public documentation (see issue #1037): describe the cleanup after a game/session:

    - Sit down the robot.
    - Closing the tmux session.
    - Copying and then deleting the recorded rosbag from the robot.

