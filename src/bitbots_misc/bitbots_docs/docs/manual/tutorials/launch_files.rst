==============
Launch Scripts
==============

Listed below are the most important launch files.
You can display the arguments of the launch files in the terminal by using the command
`ros2 launch <package-name> <launch-file-name>.launch -s`.
These launch files have default values, which can be overridden by using the syntax ``<argument>:=<value>``.

Launch Scripts in the ``bitbots_bringup`` Package
-------------------------------------------------

``teamplayer.launch``
~~~~~~~~~~~~~~~~~~~~~

This script is used to launch the robot for a game.
All game-relevant components including motion are started.
To do this, the motor current must be turned on at the robot.
After starting, the robot moves to the walk-ready position.

``highlevel.launch``
~~~~~~~~~~~~~~~~~~~~

This launch script starts all game-relevant components except for motion.

``motion_standalone.launch``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This launch script starts the motion and all components relevant to motion.
When this launch script is started, motors can be controlled and movements can be performed on the robot, such as walking or animations.
To do this, the motor current must be turned on.
After starting, the robot moves to the walk-ready position.

``vision_standalone.launch``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This launch script starts the vision, camera and all relevant components.

``simulator_teamplayer.launch``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This script starts the simulator and the robot software stack in simulation configuration (`use_sim_time:=true`).

``visualization.launch``
~~~~~~~~~~~~~~~~~~~~~~~~

This script starts RViz and visualizes the robot's sensor data.

``receive.launch``
~~~~~~~~~~~~~~~~~~

To receive debugging data via the UDP Bridge, this launch script must be started locally on a laptop.
This can be used together with the `visualization.launch` script to visualize the data.
