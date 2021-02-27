#!/usr/bin/env python3

import os
import subprocess
import sys

import rospkg
import rospy

import argparse

from wolfgang_webots_sim.webots_supervisor_controller import SupervisorController

parser = argparse.ArgumentParser()
parser.add_argument('--nogui', help="Deactivate gui", action='store_true')
parser.add_argument('--sim_id', help="identifier of the simulation")
parser.add_argument('--multi-robot', help="start world with a single robot", action='store_true')

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

if args.multi_robot:
    world_name = "4_bots.wbt"
else:
    world_name = "1_bot.wbt"
arguments = ["webots",
             batch,
             no_rendering,
             path + "/worlds/" + world_name]
sim_proc = subprocess.Popen(arguments)

rospy.init_node("webots_ros_supervisor", argv=['clock:=/clock'])
rospy.set_param("/webots_pid" + (args.sim_id if args.sim_id is not None else ""), str(sim_proc.pid))

os.environ["WEBOTS_PID"] = str(sim_proc.pid)
os.environ["WEBOTS_ROBOT_NAME"] = "supervisor_robot"
supervisor_controller = SupervisorController(ros_active=True)

while not rospy.is_shutdown():
    supervisor_controller.step()
