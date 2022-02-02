#!/usr/bin/env python3
import os
import subprocess
import psutil
import argparse
import time

import rospkg
import rclpy
from rclpy.node import Node

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--headless', help='Starts webots completely headless on a virtual framebuffer. '
                                           'This also automatically enables --batch and --no-rendering for webots',
                        action='store_true', default=False)
    group.add_argument('--nogui', help="Deactivate gui", action='store_true')
    parser.add_argument('--sim_id', help="identifier of the simulation", default="")
    parser.add_argument('--multi-robot', help="start world with a single robot", action='store_true')
    args, _ = parser.parse_known_args()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("wolfgang_webots_sim")

    # construct arguments with which webots is started depending on this scripts args
    extra_args = set()
    mode = "realtime"
    if args.nogui:
        mode = "fast"
        extra_args.update(["--batch", "--no-rendering"])

    if args.multi_robot:
        world_name = "4_bots.wbt"
    else:
        world_name = "1_bot.wbt"

    if args.headless:
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
    sim_proc = subprocess.Popen(cmd_with_args)

    # figure out webots pid
    if args.headless:
        webots_pid = None
        child_procs = psutil.Process(sim_proc.pid).children()
        while webots_pid is None:
            time.sleep(0.0001)
            assert sim_proc.poll() is None, "xvfb-run has exited"
            child_procs = psutil.Process(sim_proc.pid).children(recursive=True)

            # webots actually gets started with /bin/bash <path-to-webots>/webots <args> so we have to find that
            for child_proc in child_procs:
                exec_file = os.path.basename(child_proc.exe())
                if exec_file == "bash":
                    first_arg = os.path.basename(child_proc.cmdline()[1])
                    if first_arg == "webots":
                        webots_pid = child_proc.pid
                        break

    else:
        webots_pid = sim_proc.pid

    # set webots pid on rosparam server
    print(f"webots_pid is {webots_pid}")
    self.set_parameters([rclpy.parameter.Parameter("/webots_pid" + args.sim_id, rclpy.Parameter.Type.DOUBLE, str(webots_pid))])

    # join with child process
    try:
        exit(sim_proc.wait())
    except KeyboardInterrupt:
        exit(sim_proc.returncode)
