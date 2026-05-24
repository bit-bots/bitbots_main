========================
Testing the robot motion
========================

Make sure to test in order, since there are dependencies between some things.

Test Motion in Visualization
----------------------------
#. Test Animation:
    .. code-block:: bash

        ros2 launch bitbots_animation_server viz.launch
        ros2 run bitbots_animation_server run_animation.py cheering

#. Test Walk:
    .. code-block:: bash

        ros2 launch bitbots_bringup motion_standalone.launch
        # TODO Start RL motions
        ros2 run bitbots_teleop teleop_keyboard.py

    Let the robot move around in all directions and also combinations.

Test Motion on Robot
--------------------

#. Test Animation on Robot:
    .. code-block:: bash

        ros2 launch bitbots_animation_server test.launch
        ros2 run bitbots_animation_server run_animation.py cheering

#. Test Walk on Robot:
    .. code-block:: bash

        ros2 launch bitbots_bringup motion_standalone.launch
        # TODO Start RL motions
        ros2 run bitbots_teleop teleop_keyboard.py

    Let the robot move around in all directions and also combinations.
