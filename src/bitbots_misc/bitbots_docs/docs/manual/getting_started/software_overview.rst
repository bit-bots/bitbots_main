========================
Global Software Overview
========================

.. todo::
   Rework of the public documentation (see issue #1037): this page should give a
   global overview of our software. In addition to the bringup launch scripts
   below, add:

   - A high level data flow diagram of the software stack.
   - A list of all bitbots packages with a short description, grouped by topic.

Orchestrator and Teamplayer TUI
===============================

``bitbots_bringup`` contains a process orchestrator and a terminal frontend for it.

``bitbots_orchestrate`` (``pixi run orchestrate``) starts, stops and restarts the software components of the robot,
each as its own process group, and keeps their logs.
Its output is the combined log of all components.
Pass ``--autostart`` to start all enabled components right away.
``--profile <name>``, ``--set <parameter>=<value>``, ``--enable <component>`` and ``--disable <component>``
preselect what is started, see ``--help``.

``teamplayer`` (``pixi run teamplayer``) is a TUI for the orchestrator.
It configures and starts the components, shows their state and logs, and restarts or stops single components.
If no orchestrator is running, it starts one in the background and accepts the same preset flags for it.
When quitting, it asks whether to stop the orchestrator it started or to leave it running;
an orchestrator that was already running is always left running, so the TUI can be closed and reopened at any time.
``--attach`` only connects to a running orchestrator.
``--headless`` runs an orchestrator in the foreground instead of the TUI and starts all enabled components;
Ctrl-C then stops the components and the orchestrator.

Both talk over an HTTP API on a Unix socket in ``$XDG_RUNTIME_DIR/bitbots_orchestrate``.
Several orchestrators can run on one machine, e.g. for multiple simulated robots, by giving each a different ``--name``;
pass the same ``--name`` to the TUI to connect to it.
A second orchestrator with the same name refuses to start.
The routes live below ``/api/v1``, see ``orchestrate/src/server.rs`` in ``bitbots_bringup``.
For example, the current state can be queried with:

.. code-block:: bash

   curl --unix-socket $XDG_RUNTIME_DIR/bitbots_orchestrate/default.sock http://localhost/api/v1/status

The Stack File
--------------

What the orchestrator starts is declared in ``config/teamplayer.yaml`` of ``bitbots_bringup``,
not in the orchestrator itself.
It lists the stack's parameters, named profiles (e.g. ``sim``), and its components.
Each component names the launch file or executable that runs it and may declare

- ``if``: parameter values under which it is available, e.g. only on hardware,
- ``requires``: components that have to run before it is started,
- ``fallback_for``: a component it replaces while that one is disabled,
- ``locked``: that it cannot be deselected.

The ``blackboard`` section declares the global parameters (field dimensions, game settings, ...).
The orchestrator publishes them itself on the ROS node ``parameter_blackboard``:
through its parameter services, read-only, and as the latched ``global_parameter_blackboard`` message,
together with the maps of the selected field.
They are assembled from the field's configuration and the parameter files of ``bitbots_parameter_blackboard``,
see the comments in the stack file.
The field can be chosen in the TUI or with ``--set fieldname=<field>``, the available fields are offered as choices.
Game settings come from ``game_settings.yaml``, which the deploy tool writes;
they cannot be changed in the TUI.
For setups that need different values, such as several simulated robots,
pass overrides with ``--global <parameter>=<value>`` or ``PUT /api/v1/blackboard/overrides``.
``GET /api/v1/blackboard`` shows all published values and where each comes from.

To add a component, add an entry there and declare its package as ``exec_depend`` of ``bitbots_bringup``.
The tests of ``bitbots_bringup`` check that the file is consistent,
that every profile resolves without missing requirements,
and that all referenced launch files and packages exist and are declared.
The header of the file documents all fields.

Bringup Launch Scripts
======================

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
