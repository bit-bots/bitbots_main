Welcome to |project|'s documentation!
================================================

Description
-----------

The Dynup is the component of our softwarestack that handles dynamic stand-up motions.
Since bipedal robots tend to fall, it is important that they can get back onto their feet quickly.
In order to do so robustly, quintic splines and PID controllers are used.
The Dynup system currently supports 6 different modes: :code:`front, back, rise, descend, front_only` and :code:`back_only`.

A more in-depth view can be found in `Stelter, Sebastian. "Creating Dynamic Stand-Up Motions for Bipedal Robots Using Spline Interpolation." (2020).`



How it works
------------
Upon receiving a stand-up request, the Dynup system starts by generating four quintic splines, one for each arm and leg.
Using quintic splines makes sure, that the generated motion is smooth in position and velocity, and therefore does not cause harm to the robot or the environment.
Now, for the duration of the motion, the goals for each end-effector at the current time are calculated from the splines.
These are then sent to the stabilizer, which first evaluates whether stabilizing is necessary at this point of the motion.
Since the generated motions are very stable in the first half, stabilizing is disabled at that point.
If stabilizing is to be applied to the goal, two PID controllers controll the IMU error and correct the goals of the feet according to the difference in either pitch or roll.
Finally, the goal is forwarded to the IK solver, which turns the spline goal into motor goals, which can be sent to the motors.
Since the arms are lacking degrees of freedom, the IK solver is set to also consider approximate goals to make sure the motion continues, even if the goal cannot be reached exactly.

.. figure:: _static/flowchart.pdf



How to test it
--------------
To test the dynup system, simply start the test script with :code:`roslaunch bitbots_dynup test.launch sim:=true`.
The :code:`sim` parameter is only required, if the system is tested in simulation, otherwise omit it.
To execute the motion, run the following command: :code:`rosrun bitbots_dynup dummy_client.py <direction>`, replacing :code:`<direction>` with one of the six directions mentioned above.



How to debug
------------
A lot of additional information can be gathered by looking at the outputs of the :code:`/dynup/feedback` topic or by visualizing the splines in RViz by adding the :code:`/debug/dynup/received_goal` topic. If the outputs of both of those look fine, but the robot still does not move, make sure, that all topics are mapped correctly, which especially in the simulation might cause problems.

|description|

.. toctree::
   :maxdepth: 2

   cppapi/library_root
   pyapi/modules


Indices and tables
==================

* :ref:`genindex`
* |modindex|
* :ref:`search`
