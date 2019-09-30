Test motion
-----------
#. Test Kick in Visualization
	roslaunch bitbots_dynamic_kick viz.launch
        rosrun bitbots_dynamic_kick dummy_client.py
        the visualization should do a kick

#. Test Kick on Robot
	roslaunch bitbots_dynamic_kick test.launch

#. Test Dynup in Visualization

#. Test Dynup on Robot

#. Test Walk in Visualization
	roslaunch bitbots_quitic_walk quintic_walk_viz.launch
        - roslaunch bitbots_quintic_walk viz_walk.launch
			- RViz should open and show the robot
		- rosrun bitbots_teleop teleop_keyboard.py
			- you should be able to make the robot walk in rviz using the keyboard

#. Test Walk on Robot
	roslaunch bitbots_quintic_walk quintic_walk_test.launch
		- rosrun bitbots_teleop teleop_keyboard.py
		- Let the robot move around in all directions and also combination

#. Test Pathplanning in Visualization
	roslaunch bitbots_move_base pathfinding_move_base_viz.launch
		use rviz to set a goal, the robot visualization should walk there

#. Test Pathplanning on Robot
	roslaunch bitbots_move_base pathfinding_move_base_test.launch
		use rviz to set a goal, the real robot should walk there


#. Test Animation in Visualization
    roslaunch bitbots_animation_server viz.launch
        rosrun bitbots_animation_server run_animation.py cheering

#. Test Animation on Robot
    roslaunch bitbots_animation_server test.launch
        rosrun bitbots_animation_server run_animation.py cheering

#. Test Record UI
    roslaunch bitbots_bringup motion_standalone.launch
        start rqt record ui
        load and play some animations
        test other functions

#. Test Fallen
    roslaunch bitbots_bringup motion_standalone.launch
        lay the robot on the ground, it should stand up

#. Test Falling
    roslaunch bitbots_bringup motion_standalone.launch
        move the robot as if it is falling, it should do a falling animation