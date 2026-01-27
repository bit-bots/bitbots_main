Hardware Checklist (Pre-Competition)
====================================

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
* Verify ROS control torqueless and check robot model in RViz
   Run:

   ``rl bitbots_ros_control ros_control_standalone.launch torqueless_mode:=true`` and ``rl bitbots_ros_control viz_servos.launch``

* Connect hands to legs while watching the robot mode in RViz
   Run the same commands as above.
* Check for motor communication issues during startup and afterwards in the terminal
* Run T-pose script
   Run:

   ``rl bitbots_ros_control ros_control_standalone.launch`` and ``rr bitbots_ros_control pose_check.py``

* Test teleop walking
   Run:

   ``rl bitbots_bringup motion_standalone.launch`` and ``rr bitbots_teleop teleop_keyboard.py``

* Test getting up
   Run:

   ``rl bitbots_bringup motion_standalone.launch``

* Verify robot-specific walking parameters
* Perform extrinsic calibration
 Do the steps as described in `this documentation <https://docs.bit-bots.de/meta/manual/tutorials/extrinsic_calibration.html>`_.

* Check camera images for focus and proper transmission (10 Hz, low jitter)
   Run:

   ``rl bitbots_bringup vision_standalone.launch`` and ``ros2 topic hz /camera/image_proc`` and in ``rqt`` open the image view plugin.
