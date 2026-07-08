==============
Launch Scripts
==============

Listed below are the most important launch files.
You can display the arguments of the launch files in the terminal by using the command
`bl <package-name> <launch-file-name>.launch.py --help`.
These launch files have default values, which can be overridden by using the syntax ``--<argument> <value>``.

Launch Scripts in the ``bitbots_bringup`` Package
-------------------------------------------------

``teamplayer.launch.py``
~~~~~~~~~~~~~~~~~~~~~~~~

This script is used to launch the robot for a game.
All game-relevant components including motion are started.
To do this, the motor current must be turned on at the robot.
After starting, the robot moves to the walk-ready position.

``highlevel.launch.py``
~~~~~~~~~~~~~~~~~~~~~~~

This launch script starts all game-relevant components except for motion.

``motion_standalone.launch.py``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This launch script starts the motion and all components relevant to motion.
When this launch script is started, motors can be controlled and movements can be performed on the robot, such as walking or animations.
To do this, the motor current must be turned on.
After starting, the robot moves to the walk-ready position.

``vision_standalone.launch.py``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This launch script starts the vision, camera and all relevant components.

``simulator_teamplayer.launch.py``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This script starts the simulator and the robot software stack in simulation configuration (`--sim true`).

``visualization.launch.py``
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This script starts RViz and visualizes the robot's sensor data.

``receive.launch``
~~~~~~~~~~~~~~~~~~

To receive debugging data via the UDP Bridge, this launch script must be started locally on a laptop.
This can be used together with the `visualization.launch.py` script to visualize the data.
