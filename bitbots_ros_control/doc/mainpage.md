\mainpage
\htmlinclude manifest.html
<a href="./index-msg.html">Msg/Src Documentation</a>

# bitbots_ros_control
This package includes a hardware interface for the wolfgang robot platform used by the Hamburg Bit-Bots.


# Important
## In case of problems
- LEDs should blink red if there is a problem on the bus
- Use rqt_monitor to see which devices do not response

## Dependencies
You will have to use our forked versions of dynamxiel_workbench and DynamixelSDK which provide functions for reading multiple register values in a single syncRead. They can be found here:
https://github.com/bit-bots/DynamixelSDK
https://github.com/bit-bots/dynamixel-workbench

# Structure
The main point of the package is to provide a node which provides an interface to the robots hardware for other ROS nodes.
This node runs with a configurable control loop frequency. 
Each cycle it reads and writes to the bus system on which all hardware connects, using the Dynamxiel libraries for the protocol specification.

The main hardware interface is the wolfgang_hardware_interface. 
It will ping all devices (actuators/sensors) that are specified in the wolfgang.yaml on the different serial devices which are specified in the same file.
Based on the ping responses, the interfaces knows which devices are on which bus system. 
Therefore it is not necessary to specify this explicitly.
During read and write procedure, it will create a thread for each serial bus, to perform the bus communication in parallel.


## Hardware Interfaces
Each of the hardware interfaces implements an init, read, and write method following the ros_control standard.
Depending on its type it may use a ros_controller or directly topics to communicate with the rest of the software.
Most interface publish diagnostic information that can be viewed with rqt_monitor.

### bitfoot_hardware_interface
Reads foot pressure sensors. Displays in diagnostic msg if one of the straine gauges has a cable problem.

### button_hardware_interface
Reads the buttons from the IO-module. The rest of the IO module is done with the imu and leds interface.

### core_hardware_interface
Reads / writes the teensy that is installed on the CORE board. Enables reading of current voltage and current, as well as switch the servo power.

### dynamiel_servo_hardware_interface
Reads / writes the dynamixel servos. This merges multiple servo_bus_interfaces together, since the dynamixel_controller and the joint_state_controller need to have direct connection to all servos.

### imu_hardware_interface
Reads the IMU. Writes calibration and filter paramters.

### leds_hardware_interface
Write LED status.

### servo_bus_interface
Reads / writes all servos on one bus. Is used in the dynamixel_servo_hardware interface.

### wolfgang_hardware_interface
Merges all interfaces together. Deals with having multiple serial buses.


## Scripts
### foot_pressure_test.py
This script generates fake data for the foot pressure sensors with a gui to adjust the individual values of each force sensor.

Usage is printed to the console when running:
~~~ 
rosrun bitbots_ros_control foot_pressure_tester.py -h
~~~

### imu_interactive_marker
Creates an interactive marker that publishes IMU data. Used for testing without robot.

### led_error_blink
Starts a small node that listens to diagnostsic messages and starts blinking with the LEDs when there is a problem.
Is automatically launched with the main launch.

### pressure_calibration
Starts an interactive pressure sensor calibration procedure.

### send_joint_command
Small test script to send a joint command.

### test_leds
Small test script that sets LEDs.

### zero_on_button
ROS node that zeros the foot sensors when the button is pressed.
