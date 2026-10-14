Testing in Simulation
=====================

Test Motion
-----------

.. code-block:: bash

    ros2 launch bitbots_mujoco_sim simulator.launch
    ros2 launch bitbots_bringup motion_standalone.launch sim:=true

To get a map frame without running the localization, the simulator can publish it from the ground truth pose of the robot.
The fake localization needs the odometry and the parameter blackboard, which are started by the motion launch file:

.. code-block:: bash

    ros2 launch bitbots_mujoco_sim simulator.launch fake_localization:=true
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
        ros2 run game_controller_hsl sim_gamestate.py

- Start simulator_teamplayer with a *fake localization* (the map frame is computed from the ground truth pose of the simulator instead of running the particle filter):

    .. code-block:: bash

        ros2 launch bitbots_bringup simulator_teamplayer.launch localization:=false fake_localization:=true

  The fake localization assumes the robot starts on its own half and orients the map frame accordingly.
  The same arguments can be passed to ``mujoco_simulation.launch.py``, where the ground truth is bridged into each robot's domain.
