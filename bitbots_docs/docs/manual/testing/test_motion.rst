========================
Testing the robot motion
========================

Make sure to test in order, since there are dependencies between some things.

Test Motion in Visualization
----------------------------
#. Test Kick:
    .. code-block:: bash

        roslaunch bitbots_dynamic_kick viz.launch
        rosrun bitbots_dynamic_kick dummy_client.py

    The visualization should do a kick.
    It is normal, that the robot starts in init and not walkready.

#. Test Animation:
    .. code-block:: bash

        roslaunch bitbots_animation_server viz.launch
        rosrun bitbots_animation_server run_animation.py cheering

#. Test Dynup:
    .. code-block:: bash

        roslaunch bitbots_dynup viz.launch
        rosrun bitbots_dynup dummy_client.py front  # (or back)

    The visualization should show the robot doing the standup animation and the dynup.

#. Test Walk:
    .. code-block:: bash

        roslaunch bitbots_quintic_walk viz.launch
        roslaunch bitbots_quintic_walk viz_walk.launch
        rosrun bitbots_teleop teleop_keyboard.py

    You should be able to make the robot walk in RViz using the keyboard.

#. Test Pathplanning:
    .. code-block:: bash

        roslaunch bitbots_move_base pathfinding_move_base_viz.launch

    Use RViz to set a goal (click on 2D nav goal), the robot visualization should walk there.


Test Motion on Robot
--------------------

Make sure, that you tested the hardware and lowlevel software first.
Easiest way to do both, is to use the following script:
.. code-block:: bash

    rosrun bitbots_bringup check_robot.py

Alternativly, you can do it manually by following the following steps.

#. Test Kick on Robot:
    .. code-block:: bash

        roslaunch bitbots_dynamic_kick test.launch
        rosrun bitbots_dynamic_kick dummy_client.py

#. Test Animation on Robot:
    .. code-block:: bash

        roslaunch bitbots_animation_server test.launch
        rosrun bitbots_animation_server run_animation.py cheering

#. Test Record UI:
    .. code-block:: bash

        roslaunch bitbots_bringup motion_standalone.launch

    #. Start RQT ``record ui``.
    #. Load and play some animations.
    #. Test other functions.

#. Test Dynup on Robot:
    .. code-block:: bash

        roslaunch bitbots_dynup test.launch
        rosrun bitbots_dynup dummy_client.py

#. Test Walk on Robot:
    .. code-block:: bash

        roslaunch bitbots_quintic_walk test.launch
        rosrun bitbots_teleop teleop_keyboard.py

    Let the robot move around in all directions and also combinations.

#. Test Pathplanning on Robot:
    .. code-block:: bash

        roslaunch bitbots_move_base pathfinding_move_base_test.launch

    Use RViz to set a goal, the real robot should walk there.

#. Test Fallen:
    .. code-block:: bash

        roslaunch bitbots_bringup motion_standalone.launch

    Lay the robot on the ground, it should stand up.

#. Test Falling:
    .. code-block:: bash

        roslaunch bitbots_bringup motion_standalone.launch

    Move the robot, as if it is falling, it should do a falling animation.
