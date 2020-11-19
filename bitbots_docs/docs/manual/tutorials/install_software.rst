Software installation
=====================

First, you have to install Ubuntu 18.04 (Bionic Beaver) either on your system or in a virtual
machine.

Next, install our version of ROS packages from ``packages.bit-bots.de``. Just follow the
instructions on the website (if you never used ROS before, you do not have to execute the first two
commands). Then, install the required ROS packages::

    sudo apt install ros-melodic-amcl ros-melodic-controller-interface ros-melodic-controller-manager ros-melodic-controller-manager-msgs ros-melodic-desktop-full ros-melodic-gazebo-ros-control ros-melodic-hector-gazebo ros-melodic-hector-gazebo-plugins ros-melodic-imu-sensor-controller ros-melodic-joint-state-controller ros-melodic-joint-trajectory-controller ros-melodic-map-server ros-melodic-move-base ros-melodic-moveit ros-melodic-moveit-core ros-melodic-moveit-resources ros-melodic-moveit-ros-planning ros-melodic-moveit-ros-planning-interface ros-melodic-plotjuggler ros-melodic-pointcloud-to-laserscan ros-melodic-robot-controllers ros-melodic-robot-controllers-interface ros-melodic-robot-controllers-msgs ros-melodic-robot-localization ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-rqt-controller-manager ros-melodic-rqt-joint-trajectory-controller ros-melodic-yocs-velocity-smoother ros-melodic-spatio-temporal-voxel-layer ros-melodic-rviz-imu-plugin

After that, create an account on https://github.com/ and get team access to our organization
(https://github.com/bit-bots). Then, in a terminal, execute ``ssh-keygen`` and then
``cat ~/.ssh/id_rsa.pub`` and copy the text. Go back to GitHub > Settings > SSH and GPG keys. There,
click on New SSH Key and paste the result.

Now, in a terminal, execute ``git clone git@github.com:bit-bots/bitbots_meta.git`` to download our
software meta package. Confirm the host key by typing ``yes``. Then, go to the folder by executing
``cd bitbots_meta``. Use ``make pull-init`` to download all of the software.

Next, we need a catkin workspace. Execute ``mkdir -p ~/catkin_ws/src`` to create a workspace and go
there with ``cd ~/catkin_ws``. First, you have to install catkin:
``pip3 install git+https://github.com/catkin/catkin_tools.git``. Execute
``export PATH=$PATH:$HOME/.local/bin`` to setup your path environment. Use
``source /opt/ros/melodic/setup.bash`` to source your ROS environment. Execute ``catkin init`` to
initialize the workspace and ``catkin config -DPYTHON_VERSION=3`` to set the correct python
version. Now, execute ``ln -s ~/bitbots_meta src`` to link the source directory. Finally, build the
software with ``catkin build -c``.

To make the changes persistent, open your bashrc with ``nano ~/.bashrc`` and paste the following
three lines::

    export PATH=$PATH:$HOME/.local/bin
    source /opt/ros/melodic/setup.bash
    source $HOME/catkin_ws/devel/setup.bash
