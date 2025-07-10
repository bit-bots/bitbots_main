#!/usr/bin/env python3
import argparse
import os
import threading
import urllib

import rclpy
from bitbots_webots_sim.webots_supervisor_controller import SupervisorController
from rclpy.node import Node


class SupervisorNode:
    def __init__(self, simulator_port, robot_name):
        robot_name_without_spaces = urllib.parse.quote(robot_name)
        self.node = Node("supervisor_node")
        self.node.get_logger().warn(f"tcp://localhost:{simulator_port}/supervisor_robot_{robot_name_without_spaces}")

        self.node.get_logger().warn(robot_name_without_spaces)

        # if match:
        os.environ[
            "WEBOTS_CONTROLLER_URL"
        ] = f"tcp://localhost:{simulator_port}/supervisor_robot_{robot_name_without_spaces}"
        # else:
        #     os.environ["WEBOTS_CONTROLLER_URL"] = f"tcp://localhost:{simulator_port}/supervisor_robot"

        self.supervisor_controller = SupervisorController(ros_active=True, ros_node=self.node, robot_name=robot_name)
        self.node.get_logger().info("started webots ros supervisor")

    def run(self):
        while rclpy.ok():
            self.supervisor_controller.step()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-port", help="port of the simulation", default="1234")
    parser.add_argument("--robot_name", help="name of the robot that the supervisor controlls", default="amy")
    # parser.add_argument("--match", help="true if the a 1vs1 match is played", default="false")
    args, _ = parser.parse_known_args()

    rclpy.init()
    supervisor = SupervisorNode(args.sim_port, args.robot_name)  # , args.match == "true"
    thread = threading.Thread(target=rclpy.spin, args=(supervisor.node,), daemon=True)
    thread.start()
    supervisor.run()

    supervisor.node.destroy_node()
    rclpy.shutdown()
