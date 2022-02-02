#!/usr/bin/env python3
import argparse
import os
import rclpy
from rclpy.node import Node
import time
from wolfgang_webots_sim.webots_robot_controller import RobotController

parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', help="which robot should be started")
parser.add_argument('--sim_id', help="identifier of the simulation", default="")
parser.add_argument('--void-controller', action='store_true',
                    help="if true, a controller that only steps and does nothing else")
parser.add_argument('--disable-camera', action='store_true',
                    help="Turn on or off the camera (to speed up if only motion is required)")
parser.add_argument('--recognize', action='store_true',
                    help="if true, recognition is active (for training data collection)")
args, unknown = parser.parse_known_args()

rclpy.init(args=None)
pid_param_name = "/webots_pid" + args.sim_id


while not rospy.has_param(pid_param_name):
    self.get_logger().debug("Waiting for parameter " + pid_param_name + " to be set..")
    time.sleep(2.0)

webots_pid = rospy.get_param(pid_param_name)
os.environ["WEBOTS_PID"] = str(webots_pid)
os.environ["WEBOTS_ROBOT_NAME"] = args.robot_name

if args.void_controller:
    self.get_logger().debug("Starting void interface for " + args.robot_name)
    from controller import Robot
    r = Robot()
    while rclpy.ok():
        r.step(int(r.getBasicTimeStep()))
else:
    self.get_logger().debug("Starting ros interface for " + args.robot_name)
    r = RobotController(ros_active=True, do_ros_init=False,
                        recognize=args.recognize, camera_active=(not args.disable_camera))
    while rclpy.ok():
        r.step()

   r.step()

