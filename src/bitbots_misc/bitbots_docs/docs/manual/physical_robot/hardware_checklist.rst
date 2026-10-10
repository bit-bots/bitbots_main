Hardware Checklist (Pre-Competition)
====================================

.. todo::
   Rework of the public documentation (see issue #1037): fully rework this
   checklist for the PiPlus platform. Some items below refer to Wolfgang-specific
   hardware (e.g. crimped motor cables, springs) and need to be revised.

When Powered Off
----------------
* Check cables for insulation damage
* Inspect cable ties and cable management
* Check cables are correctly in their crimps
* Inspect connectors and renew hot glue if necessary
* Check 3D printed parts for cracks
* Move motors and check for gear damage or stiffness (e.g. overly long screws in joints)
* Inspect cleats are fully screwed in
* Check screws (including those that are hard to access, like under the springs) and replace any missing ones
* Inspect for head wobbling
* Check camera cables for hard bends
* Check if shoulders are bent
* Inspect PC power connectors

Before Powering On
------------------
* Ensure arms and legs are in the correct configuration (team markers outside, cables loose not coiled)

After Powering On
-----------------
* Verify torqueless mode and check the robot model in RViz
   Run on the robot:

   ``ros2 launch bitbots_bringup motion_standalone.launch torqueless_mode:=true``

   Run on the laptop:

   ``ros2 launch piplus_description standalone.launch js_pub:=false``

   The robot model in RViz should follow the actual joint states.
* Check for motor communication issues during startup and afterwards in the terminal
* Verify stiff control
   Run:

   ``ros2 launch bitbots_bringup motion_standalone.launch``

   The robot should reach its walk-ready position and hold it stiffly.

* Test teleop walking
   Run:

   ``ros2 launch bitbots_bringup motion_standalone.launch`` and ``ros2 run bitbots_teleop teleop_keyboard.py``

* Test getting up
   Run:

   ``ros2 launch bitbots_bringup motion_standalone.launch``

* Verify robot-specific walking parameters
* Perform extrinsic calibration
  Do the steps as described in :doc:`extrinsic_calibration`.

* Check camera images for focus and proper transmission (10 Hz, low jitter)
   Run:

   ``ros2 launch bitbots_bringup vision_standalone.launch`` and ``ros2 topic hz /zed/zed_node/rgb/image_rect_color`` and in ``rqt`` open the image view plugin.
