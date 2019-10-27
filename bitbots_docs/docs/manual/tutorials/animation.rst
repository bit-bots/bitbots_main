Animation Server
================

The Animation Server is responsible for playback of previously recorded keyframe animations. The server is called by the 'HCM', after the robot fell down and wants to get back up. It can also be used by the 'Behavior', e.g. for shooting.

The Animation Server is an ROS Action Server. For an action a message is sent to the Animation Server. This is similar to sending a normal 'message', the connection in this case however is not only one-way. The Action Server sends back the status of the active action as well as the result (successful or not).

Actively running animations, can be interrupted by the HCM. This ensures that, should the robot fall while shooting, the shooting animation is stopped and the robot stands up instead. An additional flag in the action is used to determine if a request/message comes from the HCM.

Animations consist of a series of keyframes. Each keyframe is a snapshot of motor positions at a certain point in time. During playback a file containing the recorded keyframes is read and each frame is played one after the other at a given interval. To achieve a fluid motion the frequency of interpolation is set to 200 Hz with the help of quintic splines. This interpolation is done in the 'Joint Space' (inbetween the motor positions, not inbetween the actual positions of the robot's extremities in the Cartesian space), due to the development of the project and better usability.

Animations can be run manually with `rosrun bitbots_animation_server run_animation <name>`.
All animations can be found in the package `wolfgang_animations`.

If an animation fails to run, the first thing to check is, if the HCM outputs a different 'Robot State' than 'Controlable' or 'Walking'.
Animations can only be played if the robot is in one of these two states.
