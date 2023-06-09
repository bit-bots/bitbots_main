Testing in Simulation
=====================

Test Behavior
-------------
- Test the behavior independent from vision:
    .. code-block:: bash

        ros2 launch bitbots_bringup simulator_teamplayer.launch


Test Motion
-----------
.. code-block:: bash

    ros2 launch wolfgang_webots_sim simulation.launch
    ros2 launch bitbots_bringup motion_standalone.launch sim:=true

To control walking of the roboter teleop needs to be startet  as well:
.. code-block:: bash

    ros2 run bitbots_teleop teleop_keyboard.py


Test Imu in RViz
----------------
.. code-block:: bash

    roslaunch bitbots_ros_control rviz_interactive_imu.launch
