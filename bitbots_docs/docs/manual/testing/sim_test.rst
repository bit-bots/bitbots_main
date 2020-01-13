Testing in Simulation
=====================

Test Behavior
-------------
- Test the behavior in the visualization
   - ``roslaunch humanoid_league_interactive_marker rviz_behavior_test.launch`` to start rViz, markers and dependencies
   - ``roslaunch bitbots_body_behavior behavior.launch`` to start body and head behavior
   - ``rosrun humanoid_league_game_controller sim_gamestate.py`` to simulate the game controller

- Test the behavior independent from vision or walking
    ``roslaunch bitbots_bringup simulator_teamplayer.launch use_fake_walk:=true use_fake_vision:=true``

- Test behavior with vision
    ``roslaunch bitbots_bringup simulator_teamplayer.launch use_fake_walk:=true``

Test Motion
-----------

- ``roslaunch bitbots_bringup simulator.launch``
- ``roslaunch bitbots_bringup motion.launch sim:=true``


Test Imu in RViz
----------------

``roslaunch bitbots_ros_control rviz_interactive_imu.launch``