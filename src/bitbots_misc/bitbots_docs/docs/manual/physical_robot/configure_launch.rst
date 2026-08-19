Configure & Launch a Robot
==========================

This section describes how to configure and deploy your current software to a
robot for game preparation.

.. note::
   See :doc:`starting_robot` for the prerequisites and safety warnings on when
   it is safe to start the robot and how the software is synced and built.

.. note::
   See :doc:`robots` for the robot names and IP addresses, and :doc:`setup_robot`
   for the one-time Ansible provisioning of a robot.

Deployment
----------

Deployment is the process of preparing a robot for the next game and starting the correct software.

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
   See :doc:`connecting` for how to connect and how to use tmux.

#. **Profit!**
   The robot is now ready play!

Cleanup
-------

.. todo::
   Rework of the public documentation (see issue #1037): describe the cleanup
   after a game/session:

   - Closing the tmux session.
   - Copying and then deleting the recorded rosbag from the robot.
