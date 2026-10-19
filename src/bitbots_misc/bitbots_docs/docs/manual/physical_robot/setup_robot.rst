Setting up a Robot
==================

.. note::
   These Ansible setup steps configure a robot's operating system from scratch.
   They are only relevant when provisioning a new robot or reinstalling one and
   will not be useful for most users. If you only want to deploy and run the
   software on an already configured robot, see :doc:`configure_launch`.

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
   See :doc:`robots` for the robot names and IP addresses.
