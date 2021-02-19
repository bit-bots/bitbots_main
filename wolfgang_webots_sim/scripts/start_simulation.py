#!/usr/bin/env python3

import os
import subprocess
import sys

import rospkg
import rospy

import argparse

from wolfgang_webots_sim.utils import fix_webots_folder
from wolfgang_webots_sim.webots_controller import WebotsController
from wolfgang_webots_sim.webots_robot_controller import RobotController
from wolfgang_webots_sim.webots_supervisor_controller import SupervisorController

parser = argparse.ArgumentParser()
parser.add_argument('--nogui', help="Deactivate gui", action='store_true')
args, unknown = parser.parse_known_args()

rospack = rospkg.RosPack()
path = rospack.get_path("wolfgang_webots_sim")

if args.nogui:
    mode = "fast"
    batch = "--batch"
else:
    mode = "normal"
    batch = ""

arguments = ["webots",
             batch,
             path +"/worlds/flat_world.wbt"]
sim_proc = subprocess.Popen(arguments)

os.environ["WEBOTS_PID"] = str(sim_proc.pid)
fix_webots_folder(sim_proc.pid)

robot_names = ["amy", "rory", "jack", "donna", "melody"]
for i in range(2):
    sub_processes = subprocess.Popen(["python3", os.path.dirname(sys.argv[0]) + "/single_robot.py", "--robot_name", robot_names[i]])

os.environ["WEBOTS_ROBOT_NAME"] = "supervisor_robot"
supervisor_controller = SupervisorController(True)

while not rospy.is_shutdown():
    supervisor_controller.step()
