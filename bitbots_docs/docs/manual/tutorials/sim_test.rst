Testing in Simulation
=====================

Test Behavior
-------------
#. Test the behavior independent from vision or walking
    roslaunch bitbots_bringup simulator_teamplayer.launch use_fake_walk:=true use_fake_vision:=true

#. Test behavior with vision
    roslaunch bitbots_bringup simulator_teamplayer.launch use_fake_walk:=true



Test Motion
-----------

roslaunch bitbots_bringup simulator.launch
roslaunch bitbots_bringup motion.launch sim:=true