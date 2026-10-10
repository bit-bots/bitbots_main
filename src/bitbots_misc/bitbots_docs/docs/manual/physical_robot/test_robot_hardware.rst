================================================
Testing the robot hardware and lowlevel software
================================================

Do the test in the provided order, to find out which part is faulty.

Preliminaries
-------------

#. Deploy the latest software to the robot
#. Put robot in a safe spot, e.g. on a rope hanging from the ceiling
#. Check if all cables are correctly connected
#. Open the runtime monitor in rqt, it will provide a lot of information

Manual procedure
~~~~~~~~~~~~~~~~

#. Test IMU
    Start the lowlevel software on the robot.
    Check that ``/imu/data`` is published and contains plausible values, e.g. with ``ros2 topic echo /imu/data`` or plotjuggler.

#. Test servos without torque
    Start the motion stack without torque:

    ``ros2 launch bitbots_bringup motion_standalone.launch torqueless_mode:=true``

    - the servos should be torqueless (not stiff)
    - start the visualization on your laptop
      ``ros2 launch piplus_description standalone.launch js_pub:=false``
      you should see the robot model follow the actual joint states
    - move the robot around to see if it behaves correctly
    - start the runtime monitor in rqt to check voltage, temperature and error status
    - maybe use plotjuggler to see the joint values in more detail

#. Test servos with torque
    Start the motion stack with torque:

    ``ros2 launch bitbots_bringup motion_standalone.launch``

    - it should start without any errors
    - the robot should reach its walk-ready position and hold it stiffly
    - run a short animation, e.g. ``ros2 run bitbots_animation_server run_animation.py cheering``, to verify that the joints are controlled correctly
