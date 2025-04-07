#!/usr/bin/env python3
import argparse
import os
import threading

import rclpy
from bitbots_webots_sim.webots_supervisor_controller import SupervisorController
from rclpy.node import Node


class SupervisorNode:
    def __init__(self, simulator_port):
        self.node = Node("supervisor_node")

        os.environ["WEBOTS_CONTROLLER_URL"] = f"tcp://localhost:{simulator_port}/supervisor_robot"

        self.supervisor_controller = SupervisorController(ros_active=True, ros_node=self.node)
        self.node.get_logger().info("started webots ros supervisor")

    def run(self):
        while rclpy.ok():
            self.supervisor_controller.step()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-port", help="port of the simulation", default="1234")
    args, _ = parser.parse_known_args()

    rclpy.init()
    supervisor = SupervisorNode(args.sim_port)
    thread = threading.Thread(target=rclpy.spin, args=(supervisor.node,), daemon=True)
    thread.start()
    supervisor.run()

    supervisor.node.destroy_node()
    rclpy.shutdown()
