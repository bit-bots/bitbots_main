========================
Testing the robot motion
========================

Make sure to test in order, since there are dependencies between some things.

Test Motion in Visualization
----------------------------
#. Test Kick:
    .. code-block:: bash

        ros2 launch bitbots_dynamic_kick viz.launch
        ros2 run bitbots_dynamic_kick dummy_client.py

    The visualization should do a kick.
    It is normal, that the robot starts in init and not walkready.

#. Test Animation:
    .. code-block:: bash

        ros2 launch bitbots_animation_server viz.launch
        ros2 run bitbots_animation_server run_animation.py cheering

#. Test Dynup:
    .. code-block:: bash

        ros2 launch bitbots_dynup viz.launch
        ros2 run bitbots_dynup dummy_client.py front  # (or back)

    The visualization should show the robot doing the standup animation and the dynup.

#. Test Walk:
    .. code-block:: bash

        ros2 launch bitbots_quintic_walk viz.launch
        ros2 launch bitbots_quintic_walk viz_walk.launch
        ros2 run bitbots_teleop teleop_keyboard.py

    You should be able to make the robot walk in RViz using the keyboard.


Test Motion on Robot
--------------------

Make sure, that you tested the hardware and lowlevel software first.
Easiest way to do both, is to use the following script:
.. code-block:: bash

    ros2 run bitbots_bringup check_robot.py

Alternatively, you can do it manually by following the following steps.

#. Test Kick on Robot:
    .. code-block:: bash

        ros2 launch bitbots_dynamic_kick test.launch
        ros2 run bitbots_dynamic_kick dummy_client.py

#. Test Animation on Robot:
    .. code-block:: bash

        ros2 launch bitbots_animation_server test.launch
        ros2 run bitbots_animation_server run_animation.py cheering

#. Test Record UI:
    .. code-block:: bash

        ros2 launch bitbots_bringup motion_standalone.launch

    #. Start RQT ``record ui``.
    #. Load and play some animations.
    #. Test other functions.

#. Test Dynup on Robot:
    .. code-block:: bash

        ros2 launch bitbots_dynup test.launch
        ros2 run bitbots_dynup dummy_client.py

#. Test Walk on Robot:
    .. code-block:: bash

        ros2 launch bitbots_quintic_walk test.launch
        ros2 run bitbots_teleop teleop_keyboard.py

    Let the robot move around in all directions and also combinations.

#. Test Falling:
    .. code-block:: bash

        ros2 launch bitbots_bringup motion_standalone.launch

    Move the robot, as if it is falling, it should do a falling animation.
