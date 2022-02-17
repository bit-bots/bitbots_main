Software installation with ROS2
===============================

In this tutorial, we will learn how to install ROS2 Rolling Ridley on Ubuntu 20.04 and build our software stack.

**0. Use Ubuntu 20.04**

As ROS works best on Ubuntu, we are using this distribution.
Currently, ROS 2 Rolling runs on Ubuntu 20.04.
If you are not already using Ubuntu 20.04, consider installing it on your system (perhaps as a dual boot), alternately you can run it in a virtual machine (not recommended, as recently we had so with it; https://www.virtualbox.org/) or use the ROS 2 docker (https://github.com/timonegk/rosdocked)

**1. Setup and Install ROS 2**

- Follow this guide and when it comes to the section **Install ROS 2 packages**, install the recommended `ros-rolling-desktop`: https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html
- Install additional dependencies: ``sudo apt install libfmt-dev librange-v3-dev liburdfdom-dev python3-colcon-common-extensions ros-rolling-backward-ros ros-rolling-desktop ros-rolling-control-msgs ros-rolling-control-toolbox ros-rolling-controller-manager ros-rolling-effort-controllers ros-rolling-joint-state-broadcaster ros-rolling-joint-trajectory-controller ros-rolling-joy ros-rolling-moveit-ros ros-rolling-moveit-ros-move-group ros-rolling-moveit-ros-planning ros-rolling-moveit-ros-planning-interface ros-rolling-moveit-ros-robot-interaction ros-rolling-moveit-simple-controller-manager ros-rolling-position-controllers ros-rolling-robot-localization ros-rolling-velocity-controllers ros-rolling-xacro``

**2. Download our software**

- Create a GitHub and Mafiasi-account, if not already done (see here for further information on this: http://doku.bit-bots.de/private/manual/dienste_accounts.html)
  Those services host our Git software repositories.
- Add your SSH key to GitHub and Gitea to access and sync our repositories
    - If you don't know what I am talking about or you don't yet have a SSH key, follow this guide: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys
    - Go to your account settings and add your SSH key (the ``.pub`` file) for `GitHub <https://github.com/settings/keys>`_ AND `Gitea <https://git.mafiasi.de/user/settings/keys>`_
- Now, you can clone (download) our main code repository (repo) called `bitbots_meta <https://github.com/bit-bots/bitbots_meta>`_:
    - Open a terminal and go to the directory where you want to download our code (typically `~/git/bitbots/`)
        - Create the directory with: ``mkdir -p ~/git/bitbots``
          This is were your source code will live and grow.
        - Move to this directory with: ``cd ~/git/bitbots``
    - Clone the code repository with: ``git clone git@github.com:bit-bots/bitbots_meta.git``
      Confirm the host key by typing ``yes``.
    - Move into the newly created directory with: ``cd bitbots_meta``
    - Clone all sub-repositories and other files by running: ``make pull-init``
    - Switch to our ROS 2 software version (as it is not the default yet):
        - Switch branch of bitbots_meta repo: ``git switch feature/ros-2-migration``
        - Switch branches of all sub-repositories: ``make switch-ros2``
        - Update all repos: ``make pull-all``
- If you want to run the robot's cameras on your system, also run the following command: ``make basler`` Confirm the host key by typing ``yes``.

**3. Install additional dependencies**

We need to install additional python packages using ``pip``, the python package manager.

- Upgrade python package manager: ``pip3 install pip -U``
- Install required python packages: ``pip3 install --user -r requirements.txt``

**4. Setup colcon workspace**

`Colcon <https://docs.ros.org/en/rolling/Tutorials/Colcon-Tutorial.html>`_ is the tool provided by ROS 2 to build and install our ROS packages, so that they can be lounched later.
The colcon workspace is where your source code gets build and where we use colcon.

- Create colcon workspace directory (typically ``~/colcon_ws/``)
    - Create directory with: ``mkdir -p ~/colcon_ws/src``
    - Link our software contained in the bitbots_meta repo to the newly created ``src`` directory with: ``ln -s ~/git/bitbots/bitbots_meta/ ~/colcon_ws/src/bitbots_meta``

**5. Final touches**

To let your system know where it should find all the ROS 2 dependencies and packages, we add a little bit of config to your ``~/.bashrc`` file, which will be run everytime you open a new terminal.

- Run the following three commands:
    - ``echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc``
    - ``echo 'source /opt/ros/rolling/setup.bash' >> ~/.bashrc``
    - ``echo 'source $HOME/colcon_ws/install/setup.bash' >> ~/.bashrc``

**TODOs**

- Migrate all ROS 1 packages to ROS 2!
- Install Webots simulator