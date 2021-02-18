#!/usr/bin/env python3

import argparse
import os
from wolfgang_webots_sim.webots_robot_controller import RobotController

parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', help="Deactivate gui")
args, unknown = parser.parse_known_args()

os.environ["WEBOTS_ROBOT_NAME"] = args.robot_name
os.environ["ROS_NAMESPACE"] = args.robot_name

r = RobotController(True)

while True:
    r.step()
