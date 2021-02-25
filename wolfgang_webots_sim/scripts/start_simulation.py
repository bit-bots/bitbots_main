#!/usr/bin/env python3

import os
import subprocess

import rospkg
import rospy

import argparse

from wolfgang_webots_sim.webots_controller import WebotsController

parser = argparse.ArgumentParser()
parser.add_argument('--nogui', help="Deactivate gui", action='store_true')
args, unknown = parser.parse_known_args()

rospack = rospkg.RosPack()
path = rospack.get_path("wolfgang_webots_sim")

if args.nogui:
    mode = "fast"
    batch = "--batch"
    no_rendering = "--no-rendering"
else:
    mode = "normal"
    batch = ""
    no_rendering = ""

arguments = ["webots",
             batch,
             no_rendering,
             path + "/worlds/flat_world.wbt"]
sim_proc = subprocess.Popen(arguments)

os.environ["WEBOTS_PID"] = str(sim_proc.pid)

robot_controller = WebotsController('', True, mode=mode)

while not rospy.is_shutdown():
    robot_controller.step()
