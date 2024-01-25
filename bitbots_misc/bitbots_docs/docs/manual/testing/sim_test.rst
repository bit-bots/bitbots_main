Testing in Simulation
=====================

Test Motion
-----------

.. code-block:: bash

    ros2 launch wolfgang_webots_sim simulation.launch
    ros2 launch bitbots_bringup motion_standalone.launch sim:=true

To control walking of the robot, teleop needs to be startet as well:

.. code-block:: bash

    ros2 run bitbots_teleop teleop_keyboard.py


Test Imu in RViz
----------------

.. code-block:: bash

    ros2 launch bitbots_ros_control rviz_interactive_imu.launch

Test the complete software stack in simulation
----------------------------------------------

- Start simulator_teamplayer *without* game controller (robot will walk in and do its thing):

    .. code-block:: bash

        ros2 launch bitbots_bringup simulator_teamplayer.launch game_controller:=false

- Start simulator_teamplayer *with* game controller (you can control the current game state):

    .. code-block:: bash

        ros2 launch bitbots_bringup simulator_teamplayer.launch
        ros2 run game_controller_hl sim_gamestate.py
