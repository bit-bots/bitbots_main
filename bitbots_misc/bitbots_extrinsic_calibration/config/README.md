# How to extrinsic Calibration
In order to adjust the calibration of the visualization you can change offset_x, offset_y and offset_z which are all parameters for the roll, pitch and jaw respectively.

1.) Start teamplayer.
```rl bitbots_bringup teamplayer.launch game_controller:=false behavior:=false```
2.) Start robot remote control.
```rr bitbots_teleop teleop_keyboard.py```
3.) Press key 1 for the right head mode.
4.) Start rviz2.
```rviz2```
5.) Open config file in bitbots_misc > bitbots_extrinsic_calibration > config
6.) Open rqt and navigate to Plugins > Configurations > Dynamic Reconfigure where you can configure the parameters.

**Remember to change Topic from <Volatile> to <Transient_Local>.**

If you change the calibration first change all parameters to 0.
Then start with the adjustment of the IMU parameters.

## IMU parameters
* has a right-handed coordinate system
<picture of coordinate system>
* changes orientation of the body
* change if the lines are more far away on one side / to the front

## camera parameters
* has a camera coordinate system
<picture of coordinate system>
* changes the camera direction
* change if its not aligned equally on both sides / front
