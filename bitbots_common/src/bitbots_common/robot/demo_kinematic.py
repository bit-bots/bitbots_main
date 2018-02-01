#!/usr/bin/python

import numpy as np
import time
import sys
from math import cos, sin

from bitbots.robot.kinematics import Robot, KinematicTask
from bitbots.ipc.ipc import SharedMemoryIPC
from bitbots.robot.pypose import PyPose
from bitbots.util import get_config
from bitbots.util.animation import play_animation
import rospy
config = get_config()
ipc = SharedMemoryIPC()
robot = Robot()
task = KinematicTask(robot)
pose = ipc.get_pose()
#pose = PyPose()
robot.update(pose)
root = 0
r_end_joint = 34
l_end_joint = 35
angle_task_joints = [15, 16]
ignore_joints = [7, 8, 17, 18]

def update(ipc, robot, id=-1, time=0.0):
    pose = ipc.get_pose()
    robot.set_angles_to_pose(pose, id, time)
    ipc.update(pose)

iteration_time = 0.5
steptime = 0.0025

stop = -1
subtask = False

if __name__ == "__main__":
    args=sys.argv
    if len(args) > 1:
        play_animation(args[1], ipc)
        time.sleep(3.5)
        robot.update(ipc.get_pose())
    #Radius in mm
    factor = 45.0
    #Kreismittelpunkt
    z_offset = 0
    if config["RobotTypeName"] == "Hambot":
        z_offset = 300
    #base_target = np.array((170, 90, -30))
    r_y = robot.get_joint_by_id(r_end_joint).get_endpoint()[1]
    l_y = robot.get_joint_by_id(l_end_joint).get_endpoint()[1]
    r_base_target = np.array((10, r_y, -290 - z_offset), dtype=np.float)
    l_base_target = np.array((10, l_y, -290 - z_offset), dtype=np.float)
    r_chain = task.create_chain(root, r_end_joint)
    l_chain = task.create_chain(root, l_end_joint)
    r_task = KinematicTask(robot)
    l_task = KinematicTask(robot)
    r_task.add_x_orientation(root, r_end_joint, 0.05, (1,0,0))
    l_task.add_x_orientation(root, l_end_joint, 0.05, (1,0,0))
    r_task.add_y_orientation(root, r_end_joint, 0.05, (0,1,0))
    l_task.add_y_orientation(root, l_end_joint, 0.05, (0,1,0))
    r_task.add_z_orientation(root, r_end_joint, 0.05, (0,0,1))
    l_task.add_z_orientation(root, l_end_joint, 0.05, (0,0,1))
    r_task.add_position(root, r_end_joint,  0.5, r_base_target)
    l_task.add_position(root, l_end_joint,  0.5, l_base_target)
    if subtask:
        r.task.set_subtask(l_task)
    for i in range(25):
        #local calculating stuff
        begin = rospy.get_time()
        current = begin
        end = begin + iteration_time
        dt = 0
        ## local stuff end
        while current < end:
            if stop == 0:
                exit(0)
            else:
                stop = stop - 1
            phase = current / float(iteration_time) * 2 * 3.14159625358
            target_diff = np.array((1.5 * factor * (cos(phase)), 0, -factor * (sin(phase))))
            r_target = r_base_target + target_diff
            l_target = l_base_target - target_diff
            r_task.update_target(r_target, 3)
            l_task.update_target(l_target, 3)
            r_task.execute(10)
            if not subtask:
                l_task.execute(10)
            print("R: soll : %s \nist:      %s" % (r_target, robot.get_joint_by_id(r_end_joint).get_endpoint().reshape((1,3))))
            print("L: soll : %s \nist:      %s" % (l_target, robot.get_joint_by_id(l_end_joint).get_endpoint().reshape((1,3))))
            update(ipc, robot, -1, steptime)
            dt = rospy.get_time() - current
            current = current + dt
            time.sleep(max(0, steptime - dt))
