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
       # move to a target pose in map frame
       set_goal_pose(0.5, 0.0, 0.0)

       # inspect robot and ball state
       current_pose = get_current_pose()
       ball_pose = get_ball_pose()

       # kick in map-frame direction in degrees
       kick_ball(0.0)

Launch body behavior with ``behavior_mode`` set to ``simple`` and provide the
script path with ``simple_behavior_script``.

Available functions
-------------------

- ``set_goal_pose(x, y, yaw_deg=0.0)``
- ``kick_ball(direction_deg, strength=3.0)``
- ``get_current_pose()``
- ``get_ball_pose()``
