#!/usr/bin/env python3

import os
import subprocess

import rospkg
import rospy

import argparse

from wolfgang_webots_sim.utils import fix_webots_folder
try:
    from wolfgang_webots_sim.webots_controller import WebotsController
except:
    print(f'Please execute "source {os.path.dirname(os.path.realpath(__file__))}/setenvs.sh" first')
    exit(0)

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

robot_controller = WebotsController('', True, mode=batch)

while not rospy.is_shutdown():
    robot_controller.step()

