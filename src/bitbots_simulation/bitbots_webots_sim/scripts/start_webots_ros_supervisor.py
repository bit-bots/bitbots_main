#!/usr/bin/env python3
import argparse
import os
import threading

import rclpy
from bitbots_webots_sim.webots_supervisor_controller import SupervisorController
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node


class SupervisorNode:
    def __init__(self, simulator_port, multi_robot=False):
        self.node = Node("supervisor_node")

        os.environ["WEBOTS_CONTROLLER_URL"] = f"ipc://{simulator_port}/supervisor_robot"

        if multi_robot == False:
            self.supervisor_controller = SupervisorController(ros_active=True, ros_node=self.node, domains=1)
        else:
            self.supervisor_controller = SupervisorController(ros_active=True, ros_node=self.node, domains=2)
            
        self.node.get_logger().info("started webots ros supervisor")

    def run(self):
        while rclpy.ok():
            self.supervisor_controller.step()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-port", help="port of the simulation", default="1234")
    parser.add_argument("--multi_robot", help="True if a real match is supposed to be simulated. Starts world with two robots.", default=False, type=bool)
    args, _ = parser.parse_known_args()

    rclpy.init()
    supervisor = SupervisorNode(args.sim_port, multi_robot=args.multi_robot)

    executor = EventsExecutor()
    executor.add_node(supervisor.node)

    thread = threading.Thread(target=executor.spin, args=(), daemon=True)
    thread.start()
    supervisor.run()

    supervisor.node.destroy_node()
