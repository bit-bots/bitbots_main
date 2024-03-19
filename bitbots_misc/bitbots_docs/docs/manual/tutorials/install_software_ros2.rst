Software installation with ROS2
===============================

In this tutorial, we will learn how to install ROS2 Iron Irwini on Ubuntu 22.04 and build our software stack.


**0. Use Ubuntu 22.04**

As ROS works best on Ubuntu, we are using this distribution.
Currently, ROS2 Iron runs on Ubuntu 22.04.
If you are not already using Ubuntu 22.04, consider installing it on your system (perhaps as a dual boot), alternately you can run it in a virtual machine (not recommended, as recently we had some issues with it; https://www.virtualbox.org/), a custom ROS2 docker setup (https://github.com/timonegk/rosdocked).

Alternatively you can use a devcontainer :doc:`vscode-dev-container`, with a preconfigured environment and follow those instructions, as these docs do not apply to the devcontainer.

**TLDR**: single command setup
------------------------------

**Prerequirements**
- running Ubuntu 22.04 environment (native, VM, or custom ROS2 docker setup)
- existing Github account and with SSH key added to your account
- root access to your system (sudo)

If you have not previously set up any of our software stack, you can use the following command to install and setup everything in one go:

.. code-block:: bash

  mkdir -p ~/git/bitbots \
    && cd ~/git/bitbots \
    && curl -fsSL https://raw.githubusercontent.com/bit-bots/bitbots_main/main/scripts/setup.sh > /tmp/setup.sh \
    && bash /tmp/setup.sh

Manual steps with in depth explanation
--------------------------------------

**1. Setup and Install ROS 2**

- Follow this guide and when it comes to the section **Install ROS 2 packages**, install the recommended ``ros-iron-desktop-full``: https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html
- Install additional dependencies:

.. code-block:: bash

  sudo apt install \
    clang-format \
    cppcheck \
    python3-colcon-clean \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-iron-plotjuggler-ros \
    ros-iron-rmw-cyclonedds-cpp \
    ros-iron-rqt-robot-monitor \
    ros-iron-rqt-runtime-monitor

- Run ``sudo rosdep init`` to initialize ``rosdep``, a tool that helps you install system dependencies for ROS packages.
- To aditionally get nice colored output from colcon, you can install the following pip packages:

.. code-block:: bash

  python3 -m pip install \
    git+https://github.com/ruffsl/colcon-clean \
    git+https://github.com/timonegk/colcon-core.git@colors \
    git+https://github.com/timonegk/colcon-notification.git@colors \
    git+https://github.com/timonegk/colcon-output.git@colors

**2. Install Webots**

Webots is a robot simulator, which we use to simulate our robots and test our software.
It is not strictly necessary to install it, but it is very useful for development and testing.
If you want to install it, you can do so by running ``make webots`` in the bitbots_main repository.

**3. Download our software**

- Create a GitHub account, if not already done (see here for further information on this: http://doku.bit-bots.de/private/manual/dienste_accounts.html)
  Those services host our Git software repositories.
- Add your SSH key to GitHub to access and sync our repositories
    - If you don't know what I am talking about or you don't yet have a SSH key, follow this guide: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys
    - Go to your account settings and add your SSH key (the ``.pub`` file) to `GitHub <https://github.com/settings/keys>`_
- Now, you can clone (download) our main code repository (repo) called `bitbots_main <https://github.com/bit-bots/bitbots_main>`_:
    - Open a terminal and go to the directory where you want to download our code (typically ``~/git/bitbots/``)
        - Create the directory with: ``mkdir -p ~/git/bitbots``
          This is were your source code will live and grow.
        - Move to this directory with: ``cd ~/git/bitbots``
    - Clone the code repository with: ``git clone git@github.com:bit-bots/bitbots_main.git``
      Confirm the host key by typing ``yes``, if asked.
    - Move into the newly created directory with: ``cd bitbots_main``
    - Clone all code and other files by running: ``make install``
      This will take a while, as it downloads all the code and other files from our repositories and additionally installs all missing dependencies (using rosdep and pip).
      Finally, it will register pre-commit hooks (automatic code-formatting and warnings), which will be run every time you commit code to our repositories.

**4. Setup colcon workspace**

`Colcon <https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_ is the tool provided by ROS 2 to build and install our ROS packages, so that they can be launched later.
The colcon workspace is where your source code gets build and where we use colcon.

- Create colcon workspace directory (typically ``~/colcon_ws/``)
    - Create directory with: ``mkdir -p ~/colcon_ws/src``
    - Link our software contained in the bitbots_main repo to the newly created ``src`` directory with: ``ln -s ~/git/bitbots/bitbots_main/ ~/colcon_ws/src/bitbots_main``

**5. Final touches**

To let your system know where it should find all the ROS 2 dependencies and packages and to add colored output etc., we add a little bit of config to your ``~/.bashrc`` file, which will be run every time you open a new terminal.
In case you are not using the bash shell, replace ``~/.bashrc`` and ``bash`` with your shell's configuration file.

- Run the following command:

.. code-block:: bash

  cat >> ~/.bashrc << EOF

  # Ignore some deprecation warnings
  export PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources

  # Limit ROS 2 communication to localhost (can be overridden when needed)
  export ROS_DOMAIN_ID=24
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

  # Set the default colcon workspace
  export COLCON_WS="\$HOME/colcon_ws"

  # Set the default log level for colcon
  export COLCON_LOG_LEVEL=30

  # Define a log layout
  export RCUTILS_COLORIZED_OUTPUT=1
  export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

  # Set the default Middleware
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

  # Load our ros plugin script containing useful functions and aliases for ROS 2 development
  if [[ -f \$COLCON_WS/src/bitbots_main/scripts/ros.plugin.sh ]]; then
    source \$COLCON_WS/src/bitbots_main/scripts/ros.plugin.sh
  fi

  # <<< bit-bots initialize <<<

  EOF

- Configure the robot hostnames, see :doc:`configure_hostnames`.

**6. Troubleshooting**

If you have some problems with your installation, like not finding any nodes or topics, referr here for some troubleshooting steps: https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html
