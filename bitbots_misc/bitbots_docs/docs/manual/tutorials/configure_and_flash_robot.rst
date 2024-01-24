Configure and Flash a robot
===========================

This section describes how to fully configure and flash a robot with a configured OS and your current software to prepare it for games.

Robots
------

.. note::
   Current status as of September 2023:

+--------+----------+-------------+-------------+
| Name   | Hostname | IP          | Camera IP   |
+========+==========+=============+=============+
| Amy    | nuc1     | 172.20.1.11 | 172.20.4.11 |
+--------+----------+-------------+-------------+
| Rory   | nuc2     | 172.20.1.12 | 172.20.4.12 |
+--------+----------+-------------+-------------+
| Jack   | nuc3     | 172.20.1.13 | 172.20.4.13 |
+--------+----------+-------------+-------------+
| Donna  | nuc4     | 172.20.1.14 | 172.20.4.14 |
+--------+----------+-------------+-------------+
| Melody | nuc5     | 172.20.1.15 | 172.20.4.15 |
+--------+----------+-------------+-------------+
| Rose   | nuc6     | 172.20.1.16 | 172.20.4.16 |
+--------+----------+-------------+-------------+

Flashing
--------

Flashing is the process of preparing a robot for the next game and starting the correct software.

.. note::
   Does DNS not resolve ``nuc*``? See :doc:`configure_hostnames` to fix this.

At a competition, follow these steps:

#. **Configure Wi-Fi networks for fields:**
   This needs to be done before the competition at the team area (see :doc:`competition_wifi`)!

#. **Checkout the latest code:**
   In your local `bitbots_main <https://github.com/bit-bots/bitbots_main>`_ repo run:

   #. Check that you are on the ``master`` branch
   #. ``git pull`` to get the latest changes
   #. ``make fresh-libs`` to clean and update all third party libraries

#. **Sync, configure, compile and launch software:**
   In the ``bitbots_main`` directory call the ``deploy_robots.py`` tool:

   .. code-block:: bash

      ./scripts/deploy_robots.py <nuc* | robot_name | ALL>

   This does the 5 following tasks:
   - Synchronize/Copy the current state of your local bitbots_main directory to the robot(s)
   - Install ROS 2 dependencies using `rosdep` on the robot(s), if internet is available
   - Configure game specific settings and the Wi-Fi connection on the robot(s)
   - Build/Compile the source code you just synchronized to the robot(s)
   - Launch the teamplayer software on the robot(s)

   If you need help with this tool, or want other options, look at `this README <https://github.com/bit-bots/bitbots_main/blob/master/scripts/README.md#deploy_robotspy>`_ for example usages or call:

   .. code-block:: bash

      ./scripts/deploy_robots.py -h

#. **Optional: Connect to the robot:**
   Simply copy-paste the command provided by the deploy-tool when its finished.

#. **CURRENTLY DISABLED: Reset foot pressure sensors:**
   Pick up the robot, so that the feet do not touch the ground.
   Long press the green button on the IMU, which resets the foot pressure sensors.

#. **Profit!**
   The robot is now ready play!


Ansible Configuration
---------------------

Requirements
~~~~~~~~~~~~

- Ability to connect via SSH to the robot(s)
- Have our `ansible repo <https://git.mafiasi.de/Bit-Bots/ansible>`_ checked out

Configure the robot OS with ansible
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Our Ansible setup is able to configure the following aspects of the robot:

- Configuration of kernel type and kernel/boot parameters
- Configuration of low level system parameters for better performance
- Setup/Configuration of the ``bitbots`` user account on the robot
- Network/IP configuration including:
   - Configuration of competition Wi-Fi networks (see :doc:`competition_wifi`)
   - Configuration of USB-Ethernet adapter as slave of a bridge interface, to allow for removal without losing the interface utilized by ros/dds
   - Configuration of custom MTU of 9000 (jumbo frames) for Ethernet connection to the basler camera
- Setup of custom apt repositories/packages (e.g. `packages.bit-bots.de <https://packages.bit-bots.de>`_ if applicable)
- Installation and configuration of ROS and DDS
- Configuration of Vulkan packages/drivers

To run the whole setup on a specific robot execute the following in the ansible repository folder:

.. code-block:: bash

  ansible-playbook ./playbooks/setup_robots.yml --ask-become-pass --limit <nuc*>

Ansible will execute the playbook with the ``bitbots`` user on the robots and will ask for its password to be able to utilize ``sudo``.

.. note::
   Does DNS not resolve ``nuc*``? See :doc:`configure_hostnames` to fix this.


LEGACY: Sync/Build the software using the ``robot_compile`` tool:
-----------------------------------------------------------------

We utilize a python script located in ``bitbots_main/scripts/robot_compile.py`` to allow doing the following:

- sync the local code of the whole ``bitbots_main`` or a single package onto a robot
- build the synced code on the robot afterwards
- automatically install required dependencies with ``rosdep install`` if the robot has an active internet connection
- clean the whole ``~/colcon_ws`` on a robot
- interactively configure the ``game_settings.yaml`` on a robot to prepare it for a game
- activate the fields Wi-Fi connection and disable all others

A full overview all the options are viewable with the ``-h`` flag.

**Exemplary commands:**

.. code-block:: bash

   # full sync/build of bitbots_main
   ./scripts/robot_compile.py nuc1

   # rm everything before full sync/compile
   ./scripts/robot_compile.py --clean-src --clean-build nuc1

   # sync/build and configure robot for game after
   ./scripts/robot_compile.py -k nuc1

   # only configure robot for game
   ./scripts/robot_compile.py -K nuc1

   # only sync/build a single package (bitbots_vision)
   ./scripts/robot_compile.py -p bitbots_vision nuc1

   # sync/clean build a single package (bitbots_vision)
   ./scripts/robot_compile.py -p bitbots_vision nuc1
