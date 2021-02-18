#!/usr/bin/env python3

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', help="which robot should be started")
args, unknown = parser.parse_known_args()

os.environ["WEBOTS_ROBOT_NAME"] = args.robot_name
os.environ["ROS_NAMESPACE"] = args.robot_name

# import only after setting environment variables such that ROS_NAMESPACE is actually used
from wolfgang_webots_sim.webots_robot_controller import RobotController

r = RobotController(True)

while True:
    r.step()
