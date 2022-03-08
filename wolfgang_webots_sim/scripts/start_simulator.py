#!/usr/bin/env python3
import os
import subprocess
import threading

import psutil
import argparse
import time

import rcl_interfaces.msg
import rclpy
from ament_index_python import get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters


class WebotsSim(Node):

    def __init__(self, nogui, multi_robot, headless, sim_id, robot_type):
        super().__init__('webots_sim')
        pkg_path = get_package_share_directory("wolfgang_webots_sim")

        # construct arguments with which webots is started depending on this scripts args
        extra_args = set()
        mode = "realtime"
        if nogui:
            mode = "fast"
            extra_args.update(["--batch", "--no-rendering"])

        if robot_type == "wolfgang":
            if multi_robot:
                world_name = "4_bots.wbt"
            else:
                world_name = "1_bot.wbt"
        else:
            world_name = f"walk_optim_{robot_type}.wbt"

        if headless:
            cmd = [
                "xvfb-run",
                "--auto-servernum",
                "webots",
            ]
            extra_args.update(["--stdout", "--stderr", "--batch", "--no-rendering"])
            mode = "fast"
        else:
            cmd = ["webots"]

        # actually start webots
        cmd_with_args = cmd + list(extra_args) + [f"--mode={mode}", f"{pkg_path}/worlds/{world_name}"]
        print(f"running {' '.join(cmd_with_args)}")
        self.sim_proc = subprocess.Popen(cmd_with_args)

        # figure out webots pid
        if headless:
            webots_pid = None
            child_procs = psutil.Process(self.sim_proc.pid).children()
            while webots_pid is None:
                time.sleep(0.0001)
                assert self.sim_proc.poll() is None, "xvfb-run has exited"
                child_procs = psutil.Process(self.sim_proc.pid).children(recursive=True)

                # webots actually gets started with /bin/bash <path-to-webots>/webots <args> so we have to find that
                for child_proc in child_procs:
                    exec_file = os.path.basename(child_proc.exe())
                    if exec_file == "bash":
                        first_arg = os.path.basename(child_proc.cmdline()[1])
                        if first_arg == "webots":
                            webots_pid = child_proc.pid
                            break
        else:
            webots_pid = self.sim_proc.pid

        # set webots pid on rosparam server
        print(f"webots_pid is {webots_pid}")
        self.declare_parameter("/webots_pid", int(webots_pid))
        blackboard_client = self.create_client(SetParameters, '/parameter_blackboard/set_parameters')
        while not blackboard_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('blackboard not available, waiting again...')
        req = SetParameters.Request()
        parameter = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(webots_pid))
        req.parameters = [Parameter(name="webots_pid" + sim_id, value=parameter)]
        future = blackboard_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def run_simulation(self):
        # join with child process
        try:
            exit(self.sim_proc.wait())
        except KeyboardInterrupt:
            exit(self.sim_proc.returncode)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--headless',
                       help='Starts webots completely headless on a virtual framebuffer. This also automatically enables --batch and --no-rendering for webots',
                       action='store_true')
    group.add_argument('--nogui', help="Deactivate gui", action='store_true')
    parser.add_argument('--sim_id', help="identifier of the simulation", default="")
    parser.add_argument('--multi-robot', help="start world with a single robot", action='store_true')
    parser.add_argument('--robot-type', help="which robot should be started", default="wolfgang")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = WebotsSim(args.nogui, args.multi_robot, args.headless, args.sim_id, args.robot_type)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run_simulation()

    node.destroy_node()
    rclpy.shutdown()
