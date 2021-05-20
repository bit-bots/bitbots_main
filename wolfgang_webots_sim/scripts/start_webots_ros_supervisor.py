#!/usr/bin/env python3
import os
import argparse
import time

import rospy

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim_id', help="identifier of the simulation", default="")
    args, _ = parser.parse_known_args()
    pid_param_name = "/webots_pid" + args.sim_id

    rospy.init_node("webots_ros_supervisor", argv=['clock:=/clock'])

    while not rospy.has_param(pid_param_name):
        rospy.logdebug("Waiting for parameter " + pid_param_name + " to be set..")
        time.sleep(2.0)

    webots_pid = rospy.get_param(pid_param_name)

    os.environ["WEBOTS_PID"] = str(webots_pid)
    os.environ["WEBOTS_ROBOT_NAME"] = "supervisor_robot"

    from wolfgang_webots_sim.webots_supervisor_controller import SupervisorController
    supervisor_controller = SupervisorController(ros_active=True)
    rospy.loginfo("started webots ros supervisor")
    while not rospy.is_shutdown():
        supervisor_controller.step()
