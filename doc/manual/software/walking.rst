.. _walking:

===============
Quintic Walking
===============

This document describes how the bitbots_quintic_walking works.


General Structure
=================

The walking consists of three main parts:

1. The ROS node (QuinticWalkingNode.cpp) which is the main class controlling the walking and connecting it to ROS
2. The walking engine (QuinticWalk.cpp) which generates cartesian poses of the flying foot and the trunk in relation to the support foot. These are based on splines.
3. The inverse kinematic which computes the necessary joint values to reach the computed cartesian poses. We use BioIK for this

.. image:: walking/QuinticWalk_structure.pdf


Preliminaries
=============

Here are some information about the things that we use.

Quintic Splines
---------------

Splines are a mathematical method to describe a curve using spline points. Have a look in the internet for more details.

We use quintic splines to be able to define the position, velocity, acceleration and jerk at each spline point. This enables us to generate smooth trajectories in the cartesian space.

We currently use an ROSified version of Rhobans spline implementation from the bitbots_splines package. The x-achsis is always the time while the y-achsis is the value of position or rotation.




The Walking Engine
==================

The walking engine consists of mainly two methods. One builds the splines for the next step and one gives the poses for trunk and flying foot at a given time point in the trajectory.

The engine uses a fixed time frame for the step which can be changed by the frequency parameter. In order to simplify the splines and make them independend from the frequency, a so called "phase time" is used. Therefore, changing the frequency just compresses or streches the splines in the time axis.
The engine handles two trajectories for the flying foot and the trunk. This results in 12 splines, since we need each 3 splines for position and 3 for rotation. Additionally there are some splines having meta data like which foot is the support foot. Each step starts with the double support phase and ends with the single support phase.

When the next step is computed, the last positions, velocities and accelerations are used as start points for all splines, meaning as the first spline point at time 0. Afterwards, other spline points are added. Thoses points are pre programmed but their x and/or y values can be influenced by either parameters or the desired step length. Usually those points are on meaningful time points like the switch between double and single support.

Since these trajectories are only build when doing a new double step, changes in parameters or walking commands are only shown when the current double step is finished.
The method which computes the poses at a given time point only takes the prebuild splines and computes the values for each of the splines using the methematical spline interpolation.

.. image:: walking/engine_step.pdf
.. image:: walking/engine_params.pdf


The Walking Node
================

The walking node subscribes to the /cmd_vel topic which provides a Twist.msg commanding the velocities in x,y and yaw. If it recieves only zeros, it will completly stop. If there are values present, the node calls regularly the engine to get the next cartesian poses. Since those poses are in the support foot frame, it translates them to the base_link frame and calls the IK to get the corresponding joint values. These are then published as new goals for the servos.
Furthermore, the node computes the walking odometry by adding up the changes to the robots pose from each step. This odometry is then published and used by the localization. Since the robot does not slide very much, the odometry is close to the truth. 
The node also publishes a debug topic where it provides information on the splines, as well as RViz marker messages.

To be able to change the parameters of the walking, the node provides a dynamic reconfigure interface. Each time a parameter is changed, the new parameters are handed to the walking engine.
