#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from bitbots_msgs.msg import FootPressure
from bitbots_msgs.srv import FootScale
from std_srvs.srv import Empty

CALIBRATION_WEIGHT = 0.850 * 9.81

rclpy.init(args=None)
node = Node("calibration_node")


node.get_logger().info("Waiting for " + "/foot_pressure_left/set_foot_zero" + "/foot_pressure_left/set_foot_scale"
              + "/foot_pressure_right/set_foot_zero" + "/foot_pressure_right/set_foot_scale")

zero_l = node.create_client(Empty, "/foot_pressure_left/set_foot_zero")
zero_r = node.create_client(Empty, "/foot_pressure_right/set_foot_zero")
scale_l = node.create_client(FootScale, "/foot_pressure_left/set_foot_scale")
scale_r = node.create_client(FootScale, "/foot_pressure_right/set_foot_scale")

zero_l.wait_for_service(1.0)
scale_l.wait_for_service(1.0)
zero_r.wait_for_service(1.0)
scale_r.wait_for_service(1.0)

node.get_logger().info("found all services")

node.get_logger().info("Welcome to the foot calibration suite. ")
node.get_logger().info("default calibration weight is {} kg. Change the code when using a different weight.".format(CALIBRATION_WEIGHT))


node.get_logger().info("Press enter when there is no load on the weight cells of the LEFT foot")
input()


request = Empty.Request()
req = zero_l.call_async(request)
node.get_logger().info("Successfully set the zero position for all sensors")
sensors = ['left front','left back','right front','right back',]

request = FootScale.Request()
request.weight = CALIBRATION_WEIGHT

node.get_logger().info("Starting calibration for LEFT foot")
for i in range(len(sensors)):
    request.sensor = i
    node.get_logger().info("Place calibration weight on " + sensors[i] + " cleat. Then press enter.")
    input()
    scale_l.call_async(request)
    node.get_logger().info("Load cell for " + sensors[i] + " calibrated")

node.get_logger().info("Press enter when there is no load on the weight cells of the RIGHT foot")
input()

request = Empty.Request()
req = zero_r.call_async(request)
node.get_logger().info("Successfully set the zero position for all sensors")
request = FootScale.Request()
request.weight = CALIBRATION_WEIGHT

node.get_logger().info("Starting calibration for RIGHT foot")
for i in range(len(sensors)):
    request.sensor = i
    node.get_logger().info("Place calibration weight on " + sensors[i] + " cleat. Then press enter.")
    input()
    scale_r.call_async(request)
    node.get_logger().info("Load cell for " + sensors[i] + " calibrated")


node.get_logger().info("Thank you for your patience. The configuration has been automatically saved by the pressure_converter already")
node.get_logger().info("Shutting down now.")

