Testing in Simulation
=====================

Test Behavior
-------------
- Test the behavior in the visualization:
    #. To start RViz, markers and dependencies:
        .. code-block:: bash

            roslaunch humanoid_league_interactive_marker rviz_behavior_test.launch

    #. To start body and head behavior:
        .. code-block:: bash

            roslaunch bitbots_body_behavior behavior.launch

    #. To simulate the game controller:
        .. code-block:: bash

            rosrun humanoid_league_game_controller sim_gamestate.py

- Test the behavior independent from vision or walking:
    .. code-block:: bash

        roslaunch bitbots_bringup simulator_teamplayer.launch use_fake_walk:=true use_fake_vision:=true

- Test behavior with vision:
    .. code-block:: bash

        roslaunch bitbots_bringup simulator_teamplayer.launch use_fake_walk:=true


Test Motion
-----------
.. code-block:: bash

    roslaunch bitbots_bringup simulator.launch
    roslaunch bitbots_bringup motion.launch sim:=true


Test Imu in RViz
----------------
.. code-block:: bash

    roslaunch bitbots_ros_control rviz_interactive_imu.launch
