Webots Simulation Testing Setup
================================

**1. Software installation**

Follow our `software installation guide <installation.html>`_ to install the software on the CL.

TODO: Install webots (see discussion in ansible issue)

**2. Run Webots Simulation**

We can start the Webots simulator with the following command:
``pixi run ros2 launch bitbots_bringup simulator_teamplayer.launch game_controller:=false``
This should start the simulation environment in the Webots simulator, while also starting all necessary
nodes of the robot software (walking, vision, etc.).
In the simulator we should see a field with a single robot.

Instead of doing `pixi run ...` you can also activate the pixi environment for the current terminal with
``pixi shell`` and then run the command without the prefix ``pixi run``.

With ``game_controller:=false`` we ensure, that the game_controller_listener is not started as well, but instead
we will simulate the current gamestate by our own script (in another terminal):
``pixi run ros2 run game_controller_hl sim_gamestate.py``

Which allows us to simulate the current gamestate and different phases of the game.
Now everything is ready for some simulation testing.

**3. Testing and Debugging**

By changing the simulated gamestate and seeing how the robot reacts, we can test our behavior.
If there are issues with the robots behavior, they most likely have to do with DSD configuration or different
parallelism handling in ROS 2.
To be able to visualize the current DSD execution, we can start ``pixi run rqt`` and in the ``Plugins`` menu select
``RoboCup -> DSD-Visualization``. This will show us the current DSD execution and can help in finding deadlocks
or behavior execution logic issues.

TODO: extend this section
