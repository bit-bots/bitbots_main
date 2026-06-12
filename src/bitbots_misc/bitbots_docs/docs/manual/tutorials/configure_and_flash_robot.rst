Configure and Deploy to a robot
===============================

This section describes how to fully configure and deploy your current software to a robot for game preparation.

Robots
------

We have multiple robots in our team, each with their own hostname and IP address.
Pi Plus Cameras do not use IP addresses, but are connected via USB.

.. note::
   Current status as of May 2026:

+----------+----------+------- --+-------------+-------------+
| Name     | Hostname | Username | IP          | Camera IP   |
+==========+==========+==========+=============+=============+
| Kalliope | nvidia   | nvidia   | not defined | N/A         |
+----------+----------+----------+-------------+-------------+
| Mickey   | nvidia   | nvidia   | not defined | N/A         |
+----------+----------+----------+-------------+-------------+
| Pink     | nvidia   | nvidia   | not defined | N/A         |
+----------+----------+----------+-------------+-------------+
| Romeo    | nvidia   | nvidia   | not defined | N/A         |
+----------+----------+----------+-------------+-------------+
| Amy      | nuc1     | bitbots  | 172.20.1.11 | 172.20.4.11 |
+----------+----------+----------+-------------+-------------+
| Rory     | nuc2     | bitbots  | 172.20.1.12 | 172.20.4.12 |
+----------+----------+----------+-------------+-------------+
| Jack     | nuc3     | bitbots  | 172.20.1.13 | 172.20.4.13 |
+----------+----------+----------+-------------+-------------+
| Donna    | nuc4     | bitbots  | 172.20.1.14 | 172.20.4.14 |
+----------+----------+----------+-------------+-------------+
| Melody   | nuc5     | bitbots  | 172.20.1.15 | 172.20.4.15 |
+----------+----------+----------+-------------+-------------+
| Rose     | nuc6     | bitbots  | 172.20.1.16 | 172.20.4.16 |
+----------+----------+----------+-------------+-------------+

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
- Installation and configuration of ROS and DDS
- Configuration of Vulkan packages/drivers

To run the whole setup on a specific robot execute the following in the ansible repository folder:

.. code-block:: bash

  ansible-playbook ./playbooks/setup_robots.yml --ask-become-pass --limit <hostname>

If you don't have access to the secret git-crypt data you can add ``--skip-tags git_crypt`` to the command.

Ansible will execute the playbook with the ``bitbots`` user on the robots and will ask for its password to be able to utilize ``sudo``.

.. note::
   Does DNS not resolve ``hostname``? See :doc:`configure_hostnames` to fix this.

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

#. **Prepare and deploy the software:**
   While the deployment laptop has Internet access, prepare the locked,
   portable robot environment:

   .. code-block:: bash

      pixi run deploy cache prepare

   On the wired robot LAN, start the interactive supervisor:

   .. code-block:: bash

      pixi run deploy

   Select one or more robots, choose the match profile and components, and apply
   the desired state. Existing teamplayer tmux sessions are adopted and can be
   attached directly from the TUI. For batch operation use
   ``pixi run deploy apply <targets>``.

   For more options, see `the deployment README <https://github.com/bit-bots/bitbots_main/blob/master/scripts/README.md>`_
   or call:

   .. code-block:: bash

      pixi run deploy --help

#. **Optional: Connect to the robot:**
   Select one robot and use the TUI's **Attach** action.

#. **CURRENTLY DISABLED: Reset foot pressure sensors:**
   Pick up the robot, so that the feet do not touch the ground.
   Long press the green button on the IMU, which resets the foot pressure sensors.

#. **Profit!**
   The robot is now ready play!
