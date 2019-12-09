\mainpage
\htmlinclude manifest.html
<a href="./index-msg.html">Msg/Src Documentation</a>

# bitbots_ros_control
This package includes a hardware interface for the wolfgang robot platform used by the Hamburg Bit-Bots.


## Scripts
### foot_pressure_test.py
This script generates fake data for the foot pressure sensors with a gui to adjust the individual values of each force sensor.

Usage is printed to the console when running:
~~~ 
rosrun bitbots_ros_control foot_pressure_tester.py -h
~~~