#!/usr/bin/env python3
import argparse
import os
import rospy
import time
from wolfgang_webots_sim.webots_robot_controller import RobotController

parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', help="which robot should be started")
parser.add_argument('--sim_id', help="identifier of the simulation")
parser.add_argument('--void-controller', action='store_true',
                    help="if true, a controller that only steps and does nothing else")
parser.add_argument('--recognize', action='store_true',
                    help="if true, recognition is active (for training data collection)")
args, unknown = parser.parse_known_args()

rospy.init_node("webots_ros_interface", argv=['clock:=/clock'])
pid_param_name = "/webots_pid" + (args.sim_id if args.sim_id is not None else "")


while not rospy.has_param(pid_param_name):
    rospy.logdebug("Waiting for parameter " + pid_param_name + " to be set..")
    time.sleep(2.0)

webots_pid = rospy.get_param(pid_param_name)
os.environ["WEBOTS_PID"] = webots_pid
os.environ["WEBOTS_ROBOT_NAME"] = args.robot_name

if args.void_controller:
    rospy.logdebug("Starting void interface for " + args.robot_name)
    from controller import Robot
    r = Robot()
    while not rospy.is_shutdown():
        r.step(int(r.getBasicTimeStep()))
else:
    rospy.logdebug("Starting ros interface for " + args.robot_name)
    r = RobotController(ros_active=True, do_ros_init=False, recognize=args.recognize)
    while not rospy.is_shutdown():
        r.step()

