================================================
Testing the robot hardware and lowlevel software
================================================

Do the test in the provided order, to find out which part is faulty.

Preliminaries
-------------

Do the test in the provided order, to find out which part is faulty.

#. Use robot compile to flash correct version on robot
#. Put robot in a safe spot, on a rope hanging from the ceiling
#. Check if all cables are correctly connected
#. Open diagnostic view in rqt, it will provide a lot of information
	
Test hardware and ros_control
-----------------------------

The easiest way to do this is using the semi automatic script for this. Just start it and follow the instructions

``rosrun bitbots_bringup check_robot.py``



Manual procedure
~~~~~~~~~~~~~~~~

#. Test IMU
    ``roslaunch bitbots_ros_control ros_control_standalone.launch only_imu:=true``
        - start on your laptop ``roslaunch bitbots_ros_control viz_imu.launch`` you should see the filtered orientation and an arrow showing the sum of acceleration forces
        - maybe use plotjuggler to verify raw values

#. Test pressure sensors
    motor power on and ``roslaunch bitbots_ros_control ros_control_standalone.launch only_pressure:=true``
        - start on your laptop ``roslaunch bitbots_ros_control viz_pressure.launch``
            - you should see the pressure values as arrows in rviz as well as the center of pressures
            - press on the sensors to see if they behave correctly
        - maybe use plottjuggler to get more details and see eventual drift

#. Test servos
    motor power off and ``roslaunch bitbots_ros_control ros_control_standalone.launch torqueless_mode:=true``
        - it should give you errors for not being able to ping servos
    motor power on and ``roslaunch bitbots_ros_control ros_control_standalone.launch torqueless_mode:=true``
        - it should start without any errors
        - servos should be torqueless (not stiff)
        - start on your laptop ``roslaunch bitbots_ros_control viz_servos.launch`` you should see the the robot, the TF tree and the efforts
            - move the robot around to see if it behaves correctly (ignore efforts for now, since we are torqueless)
            - start rqt diagnostic viewer, you should see all servos on green and get voltage, temperature and error status
            - maybe use plotjuggler to see the values in more detail

    turn motor power on and ``roslaunch bitbots_ros_control ros_control_standalone.launch``
        - it should start whitout any errors
        - servos should be half stiff, but still moveable
        - start on your laptop ``roslaunch bitbots_ros_control viz_servos.launch`` you should see the the robot, the TF tree and the efforts
            - run ``rosrun bitbots_ros_control send_joint_command.py``, the robot should go into init pose and be completly stiff
            - apply some force on a servo, the effort visualization should show it (if not maybe set read_effort via dynamic reconfigure to active)

