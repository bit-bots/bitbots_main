#!/usr/bin/env python3

from typing import List, Optional

import argparse
import os
import sys

from fabric import Connection, ThreadingGroup, Result

from misc import *
from tasks import Sync


class DeployRobots():
    def __init__(self):
        self._bitbots_meta_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

        self._args = self._parse_arguments()

        LOGLEVEL.CURRENT = LOGLEVEL.CURRENT + self._args.verbose - self._args.quiet

        if self._args.print_bit_bot:
            print_bit_bot()

        self._targets = self._get_targets(self._args.target)

        self.run_tasks(self._targets)

    def _parse_arguments(self) -> argparse.Namespace:
        parser = argparse.ArgumentParser(
            description="Deploy the Bit-Bots software on a robot. "
            "This script provides 5 tasks: sync, install, configure, build, launch. "
            "By default, configure and launch are disabled. You can disable tasks by using the corresponding --no-* argument."
            )

        # Positional arguments
        parser.add_argument(
            "target",
            type=str,
            help="The target robot or computer you want to compile for. Multiple targets can be specified separated by commas. 'ALL' can be used to target all known robots."
            )
        # Task arguments
        sync_group = parser.add_mutually_exclusive_group()
        sync_group.add_argument("-s", "--sync", dest="sync", action="store_true", default=True, help="Synchronize (copy) files from you to the target machine (default: True)")
        sync_group.add_argument("-S", "--no-sync", dest="sync", action="store_false", help="Disable synchronization of files (default: False)")
        
        install_group = parser.add_mutually_exclusive_group()
        install_group.add_argument("-i", "--install", dest="install", action="store_true", default=True, help="Install ROS dependencies on the target (default: True)")
        install_group.add_argument("-I", "--no-install", dest="install", action="store_false", help="Disable installation of ROS dependencies (default: False)")

        configure_group = parser.add_mutually_exclusive_group()
        configure_group.add_argument("-c", "--configure", dest="configure", action="store_true", default=False, help="Configure the target machine (default: False)")
        configure_group.add_argument("-C", "--no-configure", dest="configure", action="store_false", help="Disable configuration of the target machine (default: True)")
        
        build_group = parser.add_mutually_exclusive_group()
        build_group.add_argument("-b", "--build", dest="build", action="store_true", default=True, help="Build on the target machine (default: True)")
        build_group.add_argument("-B", "--no-build", dest="build", action="store_false", help="Disable building on the target machine (default: False)")

        launch_group = parser.add_mutually_exclusive_group()
        launch_group.add_argument("-l", "--launch", dest="launch", action="store_true", default=False, help="Launch teamplayer software on the target (default: False)")
        launch_group.add_argument("-L", "--no-launch", dest="launch", action="store_false", help="Disable launching of teamplayer software (default: True)")


        # Optional arguments
        parser.add_argument("-p", "--package", default='', help="Synchronize and build only the given ROS package")
        parser.add_argument("--clean-src", action="store_true", help="Clean source directory before syncing")
        parser.add_argument("--clean-build", action="store_true", help="Clean workspace before building. If --package is given, clean only that package")
        parser.add_argument("--print-bit-bot", action="store_true", default=False, help="Print our logo at script start")
        parser.add_argument("-v", "--verbose", action="count", default=0, help="More output")
        parser.add_argument("-q", "--quiet", action="count", default=0, help="Less output")

        return parser.parse_args()

    def _get_targets(self, input_targets: str) -> List[Target]:
        """
        Parse target argument into usable Targets.
        Targets are comma seperated and can be either hostnames, robot names or IPs
        'ALL' is a valid target and will be expanded to all known targets

        :param input_targets: Comma seperated list of targets
        :return: List of Targets
        """
        targets: List[Target] = []

        if input_targets == "ALL":
            print_info(f"Expanding 'ALL' to all known Targets: {Target._IPs.keys()}")
            for hostname in Target._IPs.keys():
                targets.append(Target(hostname))
            return targets

        for input_target in input_targets.split(","):
            try:
                target = Target(input_target)
            except ValueError:
                print_err(f"Could not determine hostname or IP from input: '{input_target}'")
                exit(1)
            targets.append(target)
        return targets

    def _get_connections(self, targets: List[Target]) -> ThreadingGroup:
        """
        Get connections to the given Targets using the 'bitbots' username.
        
        :param targets: The Targets to connect to
        :return: The connections
        """
        try:
            connections = ThreadingGroup(
                hosts=[str(target) for target in targets],
                user="bitbots",
                connection_timeout=10,
            )
            for connection in connections:
                connection.open()
        except Exception as e:
            print_err(f"Could not establish all required connections: {e}")
            sys.exit(1)
        return connections

    def run_tasks(self, targets) -> None:
        """
        TODO: Write docstring
        """
        num_tasks = sum([
            1,  # connection
            self._args.sync,
            self._args.install,
            self._args.configure,
            self._args.build,
            self._args.launch])

        current_task = 1

        # Get connection
        with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Connecting to targets via SSH", spinner="point"):
            connections = self._get_connections(self._targets)
        print_success(f"[TASK {current_task}/{num_tasks}] Connected to targets")
        current_task += 1

        # if args.sync:
        #     with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Syncing to {target.hostname}", spinner="point"):
        #         sync(target, args.package, pre_clean=args.clean_src)
        #     print_success(f"[TASK {current_task}/{num_tasks}] Synchronization of {target.hostname} successful")
        #     current_task += 1

        # if args.install:
        #     with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Installing ROS dependencies on {target.hostname}", spinner="point"):
        #         install_rosdeps(target)
        #     print_success(f"[TASK {current_task}/{num_tasks}] Installation of ROS dependencies on {target.hostname} successful")
        #     current_task += 1

        # if args.configure:
        #     # DO NOT run this in a rich concole-status, as it screws up the user input
        #     configure(target)
        #     print_success(f"[TASK {current_task}/{num_tasks}] Configuration of {target.hostname} successful")
        #     current_task += 1


        # if args.build:
        #     with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Compiling on {target.hostname}", spinner="point"):
        #         build(target, args.package, pre_clean=args.clean_build)
        #     print_success(f"[TASK {current_task}/{num_tasks}] Compilation on {target.hostname} successful")
        #     current_task += 1

        # if args.launch:
        #     with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Launching teamplayer on {target.hostname}", spinner="point"):
        #         launch_teamplayer(target)
        #     print_success(f"[TASK {current_task}/{num_tasks}] Launching teamplayer on {target.hostname} successful")
        
        # close connection
        # CURRENT_CONNECTION.close()


if __name__ == "__main__":
    try:
        DeployRobots()
    except KeyboardInterrupt:
        print_err("Interrupted by user")
        sys.exit(1)
