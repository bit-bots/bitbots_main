Simple behavior interface
=========================

The ``simple_behavior`` interface provides a beginner-friendly alternative to the
default DSD body behavior.

Usage
-----

Create a Python script with this pattern:

.. code-block:: python

   from bitbots_body_behavior.simple_behavior import *

   def main():
       # keep the head tracking the ball
       set_head_mode(HEAD_MODE_TRACK_BALL)

       # move to a target pose in map frame
       set_goal_pose(0.5, 0.0, 0.0)

       # inspect robot and ball state
       current_pose = get_current_pose()
       ball_pose = get_ball_pose()

       # stop walking and kick in map-frame direction in degrees
       stop()
       kick_ball(0.0)

Launch body behavior with one of these modes:

- ``behavior_mode: simple``: run a script from ``simple_behavior_script``.
- ``behavior_mode: simple_web``: start the local wrapper endpoint that accepts
  scripts from the Blockly page and restarts simple behavior on each submit.

Threaded execution
------------------

The ``main()`` function runs once in a background thread. This means you can
write your own loop in ``main()`` and use ``sleep()`` without blocking the ROS
timer loop of body behavior.

Practical notes:

- Keep your loop lightweight and include short sleeps with ``sleep()`` (which is
  based on simulation time), so callbacks and ROS communication stay responsive.
- ``get_current_pose()`` raises an exception if the robot pose is unavailable.
  ``get_ball_pose()`` returns ``None`` while no ball position is known.
- If ``main()`` raises an exception, the error is logged and the behavior does
  not restart ``main()`` automatically.
- Calls to helper functions are intended to be simple and frequent, but shared
  world state can change between calls.
- ``kick_ball()`` waits until the kick is finished.
- During shutdown, body behavior waits only briefly for your loop thread and
  then continues node shutdown.

Example
-------

A short "go to ball and kick" example script is available at:

``docs/simple_behavior_go_to_ball_and_kick.py``

Blockly dev interface
---------------------

For quick experimentation, there is a self-contained Blockly page at:

``docs/blockly/simple_behavior_blockly.html``

Run it with a local static file server from the package directory, for example:

.. code-block:: bash

   cd src/bitbots_behavior/bitbots_body_behavior/docs/blockly
   python3 -m http.server 8000

Then open ``http://localhost:8000/simple_behavior_blockly.html`` in a browser.

The field coordinate lookup next to the workspace shows the coordinate system
used by pose and goal functions. Select the same named field preset that was
passed to the parameter blackboard, then drag the yellow point to read its
``x`` and ``y`` coordinates. The arrow keys provide fine movement when the
point has keyboard focus. The displayed field bounds, padding, penalty-area
depth, and goal dimensions mirror the parameter-blackboard field configs.

To run scripts directly from Blockly, start body behavior in ``simple_web`` mode.
In that mode, a dedicated wrapper runs the web endpoint and starts
``body_behavior`` in ``simple`` mode for each submitted script.
By default it listens on ``http://127.0.0.1:8765`` and accepts:

- ``GET /health`` for status checks.
- ``POST /run`` with JSON body ``{"script": "...", "name": "..."}``.

When you submit a new script, the wrapper stops the currently running simple
behavior process and starts a new one with the new script.

The Blockly page's default run endpoint is set to ``/run`` on that host/port.

The toolbox separates the pupil-facing operations by purpose. Robot commands
are grouped under movement, ball actions, and head control; observations are
under sensing; game controller status is under game state; and pose construction
and calculations are under poses and field. General program flow, control,
logic, math, variables, and output have their own categories. Less commonly
needed loop controls are kept under advanced. Pose blocks use typed
connections, and the dedicated ball-position known block can guard access to a
ball pose when the ball is not currently visible.

Practical notes:

- This interface is only intended as a lightweight development helper.
- It is intentionally minimal and not hardened for production deployment.
- The web endpoint is unauthenticated and should only be exposed in trusted
  local development setups.
- Use it to generate starter scripts, then compare/refine with the hand-written
  example in ``docs/simple_behavior_go_to_ball_and_kick.py``.
- Blockly uses one pose-attribute block for ``x``, ``y``, and ``yew_deg``.
  Selecting ``yew_deg`` for a ball pose raises an attribute error because ball
  poses only contain coordinates.

Pose operations
---------------

Robot poses provide ``x``, ``y``, and ``yew_deg`` fields. The orientation is
expressed in degrees. Ball poses and poses
created with ``make_pose()`` provide ``x`` and ``y`` fields. The pose math
helpers accept either kind and operate on the map-frame coordinates:

- ``pose_distance(first, second)`` returns the distance in meters.
- ``pose_add(first, second)`` adds coordinates and returns a ball pose.
- ``pose_subtract(first, second)`` subtracts coordinates and returns a ball
  pose.
- ``pose_direction(first, second)`` returns the map-frame direction from the
  first pose to the second in degrees.
- ``pose_from_direction(origin, distance, direction_deg)`` returns a point at
  the requested distance and map-frame direction from an origin pose.

``get_ball_pose()`` returns ``None`` when the world model's ball confidence is
below the threshold configured by ``ball_max_covariance``. This applies to both
the robot's own ball estimate and ball information received from teammates.

Available functions
-------------------

- ``sleep(seconds)``
- ``set_goal_pose(x, y, yaw_deg=0.0)``
- ``set_goal_pose_from_pose(pose, yaw_deg=0.0)``
- ``kick_ball(direction_deg, strength=3.0)``
- ``set_head_mode(mode)``
- ``stop()``
- ``get_current_pose()``
- ``get_ball_pose()``
- ``get_game_state()``
- ``is_game_state(state)``
- ``is_game_stopped()``
- ``is_stopped()``
- ``has_kick()``
- ``get_seconds_remaining()``
- ``get_secondary_seconds_remaining()``
- ``get_own_score()``
- ``get_rival_score()``
- ``is_penalized()``
- ``make_pose(x, y)``
- ``pose_distance(first, second)``
- ``pose_add(first, second)``
- ``pose_subtract(first, second)``
- ``pose_direction(first, second)``
- ``pose_from_direction(origin, distance, direction_deg)``
