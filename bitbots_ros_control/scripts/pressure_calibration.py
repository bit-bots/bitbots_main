#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from bitbots_msgs.msg import FootPressure
from bitbots_msgs.srv import FootScale, FootScaleRequest, FootScaleResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

CALIBRATION_WEIGHT = 0.850 * 9.81

rclpy.init(args=None)


self.get_logger().info("Waiting for " + "/foot_pressure_left/set_foot_zero" + "/foot_pressure_left/set_foot_scale" 
              + "/foot_pressure_right/set_foot_zero" +"/foot_pressure_right/set_foot_scale")

rospy.wait_for_service("/foot_pressure_left/set_foot_zero")
rospy.wait_for_service("/foot_pressure_left/set_foot_scale")
rospy.wait_for_service("/foot_pressure_right/set_foot_zero")
rospy.wait_for_service("/foot_pressure_right/set_foot_scale")
self.get_logger().info("found all services")

self.get_logger().info("Welcome to the foot calibration suite. ")
self.get_logger().info("default calibration weight is {} kg. Change the code when using a different weight.".format(CALIBRATION_WEIGHT))


self.get_logger().info("Press enter when there is no load on the weight cells of the LEFT foot")
input()


zero = self.create_client(Empty, "/foot_pressure_left/set_foot_zero")
if zero():
    self.get_logger().info("Successfully set the zero position for all sensors")
else:
    self.get_logger().info("There was an error :(")
    exit(1)



sensors = ['left front','left back','right front','right back',]

scale = self.create_client(FootScale, "/foot_pressure_left/set_foot_scale")
request = FootScaleRequest()
request.weight = CALIBRATION_WEIGHT

self.get_logger().info("Starting calibration for LEFT foot")
for i in range(len(sensors)):
    request.sensor = i
    self.get_logger().info("Place calibration weight on " + sensors[i] + " cleat. Then press enter.")
    input()
    scale(request)
    self.get_logger().info("Load cell for " + sensors[i] + " calibrated")



self.get_logger().info("Press enter when there is no load on the weight cells of the RIGHT foot")
input()
zero = self.create_client(Empty, "/foot_pressure_right/set_foot_zero")
if zero():
    self.get_logger().info("Successfully set the zero position for all sensors")
else:
    self.get_logger().info("There was an error :(")
    exit(1)

scale = self.create_client(FootScale, "/foot_pressure_right/set_foot_scale")

self.get_logger().info("Starting calibration for RIGHT foot")
for i in range(len(sensors)):
    request.sensor = i
    self.get_logger().info("Place calibration weight on " + sensors[i] + " cleat. Then press enter.")
    input()
    scale(request)
    self.get_logger().info("Load cell for " + sensors[i] + " calibrated")


self.get_logger().info("Thank you for your patience. The configuration has been automatically saved by the pressure_converter already")
self.get_logger().info("Shutting down now.")
