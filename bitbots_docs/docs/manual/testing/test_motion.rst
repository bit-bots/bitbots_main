========================
Testing the robot motion
========================

Make sure to test in order, since there are dependencies between some things.

Test Motion in Visualization
----------------------------
#. Test Kick
	roslaunch bitbots_dynamic_kick viz.launch
        rosrun bitbots_dynamic_kick dummy_client.py
            the visualization should do a kick
            it is normal, that the robot starts in init and not walkready

#. Test Animation
    roslaunch bitbots_animation_server viz.launch
        rosrun bitbots_animation_server run_animation.py cheering

#. Test Dynup
    roslaunch bitbots_dynup viz.launch
        rosrun ''bitbots_dynup dummy_client.py front'' (or back)
        the visualization should show the robot doing the standup animation and the dynup

#. Test Walk
	roslaunch bitbots_quintic_walk viz.launch
        roslaunch bitbots_quintic_walk viz_walk.launch
		rosrun bitbots_teleop teleop_keyboard.py
            you should be able to make the robot walk in rviz using the keyboard

#. Test Pathplanning
	roslaunch bitbots_move_base pathfinding_move_base_viz.launch
		use rviz to set a goal (click on 2D nav goal), the robot visualization should walk there


Test Motion on Robot
--------------------

Make sure that you tested the hardware and lowlevel software first.
Easiest way to do both is using following script

``rosrun bitbots_bringup check_robot.py``

Alternativly you can do it manually by following the following steps

#. Test Kick on Robot
	roslaunch bitbots_dynamic_kick test.launch
        rosrun bitbots_dynamic_kick dummy_client.py

#. Test Animation on Robot
    roslaunch bitbots_animation_server test.launch
        rosrun bitbots_animation_server run_animation.py cheering

#. Test Record UI
    roslaunch bitbots_bringup motion_standalone.launch
        start rqt record ui
        load and play some animations
        test other functions

#. Test Dynup on Robot
    roslaunch bitbots_dynup test.launch
        rosrun bitbots_dynup dummy_client.py

#. Test Walk on Robot
	roslaunch bitbots_quintic_walk test.launch
		rosrun bitbots_teleop teleop_keyboard.py
		Let the robot move around in all directions and also combination

#. Test Pathplanning on Robot
	roslaunch bitbots_move_base pathfinding_move_base_test.launch
		use rviz to set a goal, the real robot should walk there

#. Test Fallen
    roslaunch bitbots_bringup motion_standalone.launch
        lay the robot on the ground, it should stand up

#. Test Falling
    roslaunch bitbots_bringup motion_standalone.launch
        move the robot as if it is falling, it should do a falling animation