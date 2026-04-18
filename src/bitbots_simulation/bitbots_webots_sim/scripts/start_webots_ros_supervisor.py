#!/usr/bin/env python3
import argparse
import os
import signal
import threading

import rclpy
from bitbots_webots_sim.webots_supervisor_controller import SupervisorController
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node


class SupervisorNode:
    def __init__(self, simulator_port):
        self.node = Node("supervisor_node")

        os.environ["WEBOTS_CONTROLLER_URL"] = f"ipc://{simulator_port}/supervisor_robot"

        self.supervisor_controller = SupervisorController(ros_active=True, ros_node=self.node)
        self.node.get_logger().info("started webots ros supervisor")

    def run(self):
        try:
            while rclpy.ok():
                self.supervisor_controller.step()
        finally:
            self.supervisor_controller.on_shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-port", help="port of the simulation", default="1234")
    args, _ = parser.parse_known_args()

    rclpy.init()
    supervisor = SupervisorNode(args.sim_port)

    executor = EventsExecutor()
    executor.add_node(supervisor.node)

    thread = threading.Thread(target=executor.spin, args=(), daemon=True)
    thread.start()
    # We need to reset the handler because the webots controller API overwrites it.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    supervisor.run()

    supervisor.node.destroy_node()
