#!/usr/bin/env python3

import os
import subprocess

import rospkg
import rospy

from wolfgang_webots_sim.utils import fix_webots_folder
from wolfgang_webots_sim.webots_controller import WebotsController

rospack = rospkg.RosPack()
path = rospack.get_path("wolfgang_webots_sim")

arguments = ["webots",
             "--batch",
             path +"/worlds/flat_world.wbt"]
sim_proc = subprocess.Popen(arguments)

os.environ["WEBOTS_PID"] = str(sim_proc.pid)
fix_webots_folder(sim_proc.pid)

robot_controller = WebotsController('', True)

while not rospy.is_shutdown():
    robot_controller.step()

