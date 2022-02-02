#!/usr/bin/env python3
import os
import argparse
import threading
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from wolfgang_webots_sim.webots_supervisor_controller import SupervisorController


class SupervisorNode(Node):
    def __init__(self, pid_param_name):
        super().__init__('supervisor_node')
        blackboard_client = self.create_client(GetParameters, '/parameter_blackboard/get_parameters')
        while not blackboard_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('blackboard not available, waiting again...')
        req = GetParameters.Request(names=[pid_param_name])
        while True:
            future = blackboard_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                break
            else:
                self.get_logger().info("Waiting for parameter " + pid_param_name + " to be set..")
                time.sleep(2.0)
        webots_pid = future.result().values[0]

        os.environ["WEBOTS_PID"] = str(webots_pid)
        os.environ["WEBOTS_ROBOT_NAME"] = "supervisor_robot"

        self.supervisor_controller = SupervisorController(ros_active=True)
        self.get_logger().info("started webots ros supervisor")

    def run(self):
        while rclpy.ok():
            self.supervisor_controller.step()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim_id', help="identifier of the simulation", default="")
    args, _ = parser.parse_known_args()
    pid_param_name = "webots_pid" + args.sim_id

    rclpy.init()
    node = SupervisorNode(pid_param_name)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run()

    node.destroy_node()
    rclpy.shutdown()
