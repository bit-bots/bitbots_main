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

Launch body behavior with ``behavior_mode`` set to ``simple`` and provide the
script path with ``simple_behavior_script``.

Threaded execution
------------------

The ``main()`` function runs once in a background thread. This means you can
write your own loop in ``main()`` and use ``sleep()`` without blocking the ROS
timer loop of body behavior.

Practical notes:

- Keep your loop lightweight and include short sleeps, so callbacks and ROS
  communication stay responsive.
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

Available functions
-------------------

- ``set_goal_pose(x, y, yaw_deg=0.0)``
- ``kick_ball(direction_deg, strength=3.0)``
- ``set_head_mode(mode)``
- ``stop()``
- ``get_current_pose()``
- ``get_ball_pose()``
