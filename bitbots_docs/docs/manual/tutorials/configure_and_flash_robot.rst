Configure and Flash a robot
===========================

This section describes how to fully configure and flash a robot with configured OS and your current software 
to prepare it for games.


Robots
------
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


Flashing TLDR
-------------

At a competition follow these steps:

#. **Configure Wi-Fi**
This needs to be done before any game at the team area!
Either do it with `ansible` or ask Timon/Joern.

#. **Checkout the latest code**
In your local `bitbots_meta <https://github.com/bit-bots/bitbots_meta>`_ repo run:
- ``make status`` to check if the submodules are on the correct branch (normally ``master``)
- ``make pull-all`` to update all submodules

#. **Sync sources, build them on robot and configure field Wi-Fi**
In the ``bitbots_meta`` repo run the robot compile script:

.. code-block:: bash

  ./scripts/robot_compile.py -k nuc*

The ``-k`` flag also runs the configuration of ``game_settings.yaml`` and Wi-Fi after sync/compilation.

#. **SSH onto the robot**
.. code-block:: bash

  # use IP if there is no DNS resolution
  ssh bitbotsd@nuc*

#. **Set IP of visualization laptop**
On the robot configure the ``target_ips`` field in ``~/colcon_ws/src/udp_bridge/config/udp_bridge.yaml```.
Set it to the IP of the visualization laptop, which needs to also be connected to the field Wi-Fi.

#. **Launch ``teamplayer.launch``**
Start a screen session by running ``screen``.
In the session start our software with ``rl bitbots_bringup teamplayer.launch`.

#. **Reset foot pressure sensors**
Pick up the robot, so that the feet do not touch the ground.
Long press the green unpenalize button on the IMU, which resets the foot pressure sensors.

**The robot is now ready play!**


Full Configuration
------------------

Requirements
~~~~~~~~~~~~

- have a local ROS setup including a local checked out `bitbots_meta <https://github.com/bit-bots/bitbots_meta>`_ repository (including your changes) 
- optionally have our `ansible repo <https://git.mafiasi.de/Bit-Bots/ansible>`_ checked out
- ability to SSH into the robot

Configure the robot OS with ansible (optional)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you need to setup/reconfigure one of the following aspects of the robot and do not want to or know how to do it manually, this step is required:

- configuration of kernel type and kernel/boot parameters
- configuration of low level system parameters for better performance
- setup/configuration of the bitbots user account on the robot
- network/IP configuration including:
  - configuration of USB-Ethernet adapter as slave of a bridge interface, to allow for removal without losing the interface utilized by ros/dds
  - configuration of custom MTU of 9000 (jumbo frames) for ethernet connection to the basler camera
- setup of custom apt repositories/packages (e.g. `packages.bit-bots.de <https://packages.bit-bots.de>`_)
- installation and configuration of ros/dds
- configuration of vulkan packages/drivers

To run the whole setup on a specific robot (amy or ``nuc1`` in this example) execute the following in the ansible repository folder:

.. code-block:: bash

  ansible-playbook ./playbooks/setup_robots.yml --ask-become-pass --limit nuc1

Ansible will execute the playbook with the ``bitbots`` user on the robots and will ask for its password to be able to utilize ``sudo``. 

**HINT**: If you are in a situation, where there is no DNS server resolving ``nuc*`` (e.g. on a competition), you might need to add the robot IPs to your ``/etc/hosts``

.. code:: bash

  172.20.1.11     nuc1 amy
  172.20.1.12     nuc2 rory
  172.20.1.13     nuc3 jack
  172.20.1.14     nuc4 donna
  172.20.1.15     nuc5 melody
  172.20.1.16     nuc6 rose


Sync/Build the software
-------------------------

We utilize a python script located in ``bitbots_meta/scripts/robot_compile.py`` to allow doing the following:
- sync the local code of the whole ``bitbots_meta`` or a single package onto a robot
- build the synced code on the robot afterwards
- automatically install required dependencies with ``rosdep install`` if the robot has an active internet connection
- clean the whole ``~/colcon_ws`` on a robot 
- interactively configure the ``game_settings.yaml`` on a robot to prepare it for a game
- activate the fields Wi-Fi connection and disable all others

A full overview all the options are viewable with the ``-h`` flag. 

**Exemplary commands from ``bitbots_meta``**
.. code-block:: bash

  # full sync/build of bitbots_meta
  ./scripts/robot_comile.py nuc1

  # rm everything before full sync/compile
  ./scripts/robot_comile.py --clean-src --clean-build nuc1

  # sync/build and configure robot for game after
  ./scripts/robot_comile.py -k nuc1

  # only configure robot for game
  ./scripts/robot_comile.py -K nuc1

  # only sync/build a single package (bitbots_vision)
  ./scripts/robot_comile.py -p bitbots_vision nuc1

  # sync/clean build a single package (bitbots_vision)
  ./scripts/robot_comile.py -p bitbots_vision nuc1


Configuration of the Robot
--------------------------

**Wi-Fi configuration**
At a competition there will be different wifi network for the fields, these can be setup with our ansible playbook for the robots.
This is done by editing the ``group_vars/robots.yml`` config variables e.g.:

.. code-block:: yaml
  # To configure competition wifi uncomment the lines below 
  # configure team_number,  connection_name (ssid), connection_password, ip/gateway
  # and run ansible-playbook ./playbooks/setup_robots.yml --tags competition_wifi.

  team_number: 6
  network_configure_competition_wifi: true
  network_competiton_wifi_connections:
    - connection_name: competition_field_a_ssid
      connection_password: RoboCup2023
      ip: "192.168.0.{{ team_number }}{{ player_number }}"
      gateway: 192.168.0.1
    - connection_name: competition_field_b_ssid
      connection_password: RoboCup2023
      ip: "192.168.0.{{ team_number }}{{ player_number }}"
      gateway: 192.168.0.1

Then run ``ansible-playbook ./playbooks/setup_robots.yml --tags competition_wifi`` to apply this configuration.

To configure each robot before a game to actually use the correct: 
- ``player_number``
- ``team_number``
- ``position``
- ``wifi_connection``
- etc.

we again utilize the ``robot_comile.py`` script with either ``-k/-K`` switches, as described in above examples.
This starts an interactive dialog for configuration and also allows us to only enable a single wifi connection, which will be used for the game.

**Package configuration**
With the packages ``udp_bridge`` as well as ``humanoid_league_team_communication``, we have two packages which are dependent on the network configuration.
For this reason they currently need to be configured manually.
