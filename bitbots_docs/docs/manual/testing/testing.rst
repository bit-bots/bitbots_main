===========
How to Test
===========

General Remarks
===============

We want to test our software in a sensible manner because it is important that our software reliably works in a game.
A structured approach for testing offers a higher probability of success and is faster (so there is time left for tasks which are fun).

Tests are common in software development.
Therefore, they are a well researched field.
However, they are mostly applied to classical software development of servers and regular applications.
In robotics, the hardware and the connection to reality have to be considered as well.
So additionally to the usual problems, the Reality Gap influences the testing process.
The term Reality Gap describes the difference between simulation and reality.
Because of this, in simulation tested software may not necessarily work on the robot.
Still, it makes sense to use classical test methods and tests in simulation at the beginning before testing on the robot.
This saves time and protects the hardware.
Moreover, lots of mistakes can be found with this.
But it is essential to know that not all mistakes can be found with this method.

Our software is organized in packages.
Each one has a status tag which shows the current test status.
The test status of all packages is visualized in the architecture diagram.
Through the different levels an orientation is given about what has to be tested and which software already works well.
In the following, this will be explained in more detail.


Automated Testing
=================

The below guide describes how our software can be tested manually however we also use automated tests via the package
`bitbots_test`_.

.. seealso:: `bitbots_test`_ Documentation

.. _bitbots_test: http://doku.bit-bots.de/package/bitbots_test/latest/


Unit vs. Integration Testing
============================

In general, it is possible to test a package on its own, e.g., only the walking, or to test the integration of a package with several others, e.g., walking together with path planning.
Hereby, it makes sense to test first if each single component works.
After that, the integration of the components should be tested pairwise.
If this has been done successfully, the whole software stack can be tested.
This is a bottom-up approach.
Using this has the advantage that for surfacing errors it is clear which package caused them.
Another, not recommended, approach would be the top-down approach.
With this approach the whole software stack is started right at the beginning.


Testing of a Single Package
===================================

At first we will look at how we can test a single package and to which status we can then set that part in the architecture diagram if this test was successful.

Compile (compiles)
------------------

The first step is to test if the package compiles.
Obviously this should preferably be tested on the same system that is used on the robot (Ubuntu 18.04 with Melodic).
A part of this is to check if all dependencies are correct in the package.xml.
This is important so they can be installed with rosdep.

Starting (starts)
------------------

The next step is to check if the package actually starts without crashing instantly with an obvious error.
A part of this is that a launchfile exists.

Testing in Visualization (tested_viz)
----------------------------------------

The easiest real test if the software works is in visualization with fake data/commands and then to look at the output.
For this you can also use real data that was recorded in a rosbag.

For this, it is important to be aware of your own testing bias.
Humans tend to test their own software for exactly the use cases they had in mind during programming.
However it is important to explicitely check all possibilities.
This means especially to test for edge cases and on robustness, e.g., testing with wrong data and against other parts of the software stack that crash.
A good method for this is to let someone else test your software, someone who was not involved in the programming of it.

This method is essentially used to prevent errors from happening and less to see how well something works.
The tests in this stage can be done via multiple methods:

Input
^^^^^^^^^^^

1. rostopic pub
2. RViz interactive marker
    1. Path planning: inbuilt Navigation goal in RViz
    2. behavior: humanoid_league_interactive_marker
3. special test scripts:
    1. walking: bitbots_teleop
    2. behavior: sim_gamestate (in gamecontroller package)
4. rosbags

Output
^^^^^^^^^^^^

1. rostopic echo
2. RViz marker
    1. Object detection: humanoid_league_rviz_marker
    2. Walking: published its own markers
3. RViz robot model
    1. you might need to make the joint commands into joint_state (bitbots_bringup motor_goals_viz_helper.py)
4. special visualization tools for rqt
    1. DSD: dynamic_stack_decider_visualization
    2. Object Detection: humanoid_league_relative_rqt
    3. Vision: bitbots_vision_tools

Testing with simulated data (tested_simulator)
------------------------------------------------

We can test with values closer to real data if we do not use random data from humans, but with simulated data.
We mainly use Gazebo for simulation.
It offers us two possibilities.

One option is to simulate the physics (rather) accurately and let the robot actually walk.
However this is computationally intensive, even if closer to reality.

Alternatively, we can use less accurate physics and just let the robot hover the ground instead of walking.
This is especially a good option for testing the behavior,

:code:roslaunch bitbots_bringup simulator.launch
Starts the simulator.

:code:roslaunch bitbots_bringup motion.launch sim:=true
Starts the motion in the simulator (Walking, Animation, HCM)

:code:roslaunch bitbots_bringup highlevel.launch sim:=true vision:=true
Starts highlevel software (Gamecontroller, Teamcomm, Behavior, Vision, Transformer, Localization, Pathfinding)

Testing on the robot (tested_robot)
--------------------------------------

If the other test phases are completed it is time to get the real robot.
For testing this it is especially important to explicitely test the edge cases.

Integration Testing (tested_integration)
========================================

Stable Software (stable)
-------------------------
By our definition software is considered stable if it has been used in multiple games without changes since then.


What to do when changing a package?
====================================================
Even when only small changes are applied to the master branch, the package has to be tested again to keep its test status.
If the package is not tested again or only partially tested the test status has to be adapted to 'unknown' or the reached test state.


Conclusion
===============
1. Each package has to be tested on its own
    1. compiles
    2. starts
    3. using visualization
    4. using simulation
    5. on the real robot
2. test packages pairwise
3. test the complete stack (integration)
