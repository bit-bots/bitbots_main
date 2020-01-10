The HCM
=======

The HCM is the part of our software stack which handles all reflexlike behaviour of the robot, e.g. falling.
It helps abstracting from the fact that the robot has legs and can fall.
This allows the high level behavior to focus on tactics and the robot can be controlled as if it has wheels.

Since the robot can do different movements, e.g. kicking and walking, it is necessary to make sure that only one software part is controlling the joints at a given point of time.
Otherwise the motors get conflicting goals and start to shake.
If the robot falls or has to stand up, the HCM overwrites the goals from higher behaviors to make the robot stand again.

There are other possible ways to achieve the goals of the HCM:
1. Having all motion parts in a monolytic block
2. Using some kind of subsumtion logic

We decided on using the HCM approach to make it easy to exchange motion parts (each part is a single ROS node) and to have a semantic state of the robot, since it is necessary information for the high level behavior.

Tasks of the HCM
----------------

The HCM performs 6 tasks to abstract from the robot type:
1. Handling hardware problems
    If a problem with servos or sensors is detected the robot goes in a safe state and stop moving.
2. Manual stop
    Can be invoqued via service to stop the robot. Either by pressing the button or by the gamecontroller.
    Allows easier handling.
3. Fall handling
    Falls are detected and the robot is put into a safe position, e.g. making sure not to land on the head.
4. Standing up
    If the robot is lying on the floor, it stands up again automatically.
5. Joint Mutex
    The HCM makes sure that each servo is only controlled by one software part.
    It also disables control during falling and standing up.
6. Providing semantic state
    A semantic state, e.g. fallen or hardware error, is provided to the higher level software to inform about the
    current state of the robot.


How the HCM works
-----------------

The HCM uses a DSD to decide what the state of the robot is and to decide on the actions that should be performed.
This state is then published as a ROS message (`/robot_state`).
To see which state is which, you have to look at the message definition (`rosmsg show humanoid_league_msgs/RobotControlState`).

The HCM subscribes to all joint goal topics of the different software parts.
Dependend on its state, it forwards the goals or not.
Sensor data is not influenced by the HCM, since it does not need to be mutexed.

How the HCM is started
---------------------------

The easiest way to start the HCM is to launch the complete motion (`roslaunch bitbots_bringup motion_standalone.launch`).
For debugging it is sometimes better to launch the single parts by themselves.
The HCM needs the animation server (`roslaunch bitbots_animation_server animation.launch`) to work because it is needed to perform falling and stand up animations.
To be able to actually control the hardware, ros_control needs to run (`roslaunch bitbots_ros_control ros_control_standalone.launch`).
Finally launch the HCM itself (`roslaunch bitbots_hcm hcm_standalone.launch`).


What to do when it does not work
--------------------------------

1. Is `ros_control` running? Do you recieve joint states (`/joint_states`) or IMU data (`/imu/data_raw`)?
2. What is the state of the HCM (`rostopic echo /robot_state`)? The number has to be matched with the message
   description (`rosmsg show humanoid_league_msgs/RobotControlState`).
3. The visualization of the DSD is possible with the standard DSD visualization using the rqt plugin.
   There you can see exactly which decision is responsible for the current behavior and then you can look into the code.

