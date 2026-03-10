Configure and Deploy to a robot
===============================

This section describes how to fully configure and deploy your current software to a robot for game preparation.

Robots
------

We have multiple robots in our team, each with their own hostname and IP address.

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

Configuration with Ansible
--------------------------

Requirements
~~~~~~~~~~~~

- Ability to connect via SSH to the robot(s)
- Have our `ansible repo <https://github.com/bit-bots/ansible>`_ checked out

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
- Installation and configuration of ROS and DDS
- Configuration of Vulkan packages/drivers

To run the whole setup on a specific robot execute the following in the ansible repository folder:

.. code-block:: bash

  ansible-playbook ./playbooks/setup_robots.yml --ask-become-pass --limit <nuc*>

If you don't have access to the secret git-crypt data you can add ``--skip-tags git_crypt`` to the command.

Ansible will execute the playbook with the ``bitbots`` user on the robots and will ask for its password to be able to utilize ``sudo``.

.. note::
   Does DNS not resolve ``nuc*``? See :doc:`configure_hostnames` to fix this.

Deployment
----------

Deployment is the process of preparing a robot for the next game and starting the correct software.

.. note::
   Does DNS not resolve the robot name or ``nuc*``? See :doc:`configure_hostnames` to fix this.

At a competition, follow these steps:

#. **Configure Wi-Fi networks for fields:**
   This needs to be done before the competition at the team area (see :doc:`competition_wifi`)!

#. **Checkout the latest code:**
   In your local `bitbots_main <https://github.com/bit-bots/bitbots_main>`_ repo run:

   #. Check that you are on the ``main`` branch
   #. ``git pull`` to get the latest changes

#. **Sync, configure, compile and launch software:**
   In the ``bitbots_main`` directory run the deploy tool:

   .. code-block:: bash

      pixi run deploy <nuc* | robot_name | ALL>

   This does the following tasks:
   - Synchronize/Copy the current state of your local bitbots_main directory to the robot(s)
   - Install necessary dependencies on the robot(s)
   - Configure game specific settings and the Wi-Fi connection on the robot(s)
   - Build/Compile the source code you just synchronized to the robot(s)
   - Launch the teamplayer software on the robot(s)

   If you need help with this tool, or want other options, look at `this README <https://github.com/bit-bots/bitbots_main/blob/master/scripts/README.md#deploy_robotspy>`_ for example usages or call:

   .. code-block:: bash

      pixi run deploy -h

#. **Optional: Connect to the robot:**
   Simply copy-paste the command provided by the deploy-tool when its finished.

#. **CURRENTLY DISABLED: Reset foot pressure sensors:**
   Pick up the robot, so that the feet do not touch the ground.
   Long press the green button on the IMU, which resets the foot pressure sensors.

#. **Profit!**
   The robot is now ready play!
