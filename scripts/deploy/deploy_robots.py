#!/usr/bin/env python3

import argparse
import os

from misc import *
from tasks import AbstractTask, Build, Install, Launch, Sync


class DeployRobots():
    def __init__(self):
        self._bitbots_meta_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

        self._args = self._parse_arguments()

        LOGLEVEL.CURRENT = LOGLEVEL.CURRENT + self._args.verbose - self._args.quiet

        if self._args.print_bit_bot:
            print_bit_bot()

        self._targets = self._parse_targets()
        self._tasks = self._register_tasks()

        # Execute tasks on all given targets
        self.run_tasks()

    def _parse_arguments(self) -> argparse.Namespace:
        parser = argparse.ArgumentParser(
            description="Deploy the Bit-Bots software on a robot. "
            "This script provides 5 tasks: sync, install, configure, build, launch. "
            "By default, configure and launch are disabled. You can disable tasks by using the corresponding --no-* argument."
            )

        # Positional arguments
        parser.add_argument(
            "targets",
            type=str,
            help="The targets to deploy to. Multiple targets can be specified separated by commas. 'ALL' can be used to target all known robots."
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
        parser.add_argument("-u", "--user", default="bitbots", help="The user to connect to the target machines with")
        parser.add_argument("-w", "--workspace", default="~/colcon_ws", help="The workspace to deploy to")
        parser.add_argument("--clean-src", action="store_true", help="Clean source directory before syncing")
        parser.add_argument("--clean-build", action="store_true", help="Clean workspace before building. If --package is given, clean only that package")
        parser.add_argument("--connection-timeout", action="TODO", default=10, help="Timeout to establish SSH connections in seconds.")
        parser.add_argument("--print-bit-bot", action="store_true", default=False, help="Print our logo at script start")
        parser.add_argument("-v", "--verbose", action="count", default=0, help="More output")
        parser.add_argument("-q", "--quiet", action="count", default=0, help="Less output")

        return parser.parse_args()

    def _parse_targets(self) -> list[Target]:
        """
        Parse target argument into usable Targets.
        The argument is a comma seperated string of either hostnames, robot names or IPs.
        'ALL' is a valid argument and will be expanded to all known targets.

        :return: List of Targets
        """
        input_targets = self._args.targets
        targets: list[Target] = []

        if input_targets == "ALL":
            all_known_hostnames = KNOWN_TARGETS.keys()
            print_info(f"Expanding 'ALL' to all known Targets: {all_known_hostnames}")
            for hostname in all_known_hostnames:
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

    def _register_tasks(self) -> list[AbstractTask]:
        """
        Register and configure all activated tasks.

        :return: List of tasks
        """
        tasks = []

        if self._args.sync:
            tasks.append(Sync(
                self._bitbots_meta_path,
                self._args.workspace,
                self._args.package,
                self._args.clean_src,
            ))
        
        if self._args.install:
            tasks.append(Install(
                self._args.workspace,
                self._args.user,
                self._args.connection_timeout
            ))

        if self._args.configure:
            pass  # TODO

        if self._args.build:
            tasks.append(Build(
                self._args.workspace,
                self._args.package,
                self._args.clean_build,
            ))

        if self._args.launch:
            tasks.append(Launch(
                "teamplayer",
                self._args.user,
                self._args.connection_timeout
            ))

        return tasks

    def run_tasks(self) -> None:
        """
        TODO: Write docstring
        """
        num_tasks = len(self._tasks) + 1  # +1 for establishing connections to the targets
        current_task = 1  # Track current task for status output

        # Get connection
        with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Connecting to targets via SSH", spinner="point"):
            connections = get_connections_from_targets(
                self._targets,
                self._args.user,
                self._args.connection_timeout
            )
        print_success(f"[TASK {current_task}/{num_tasks}] Connected to targets")
        current_task += 1

        # Run tasks
        for task in self._tasks:
            with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] {task.__class__.__name__}", spinner="point"):
                results = task.run(connections)
            if results is None:
                print_warn(f"[TASK {current_task}/{num_tasks}] {task.__class__.__name__} returned no results.")
            if results is not None and not results.failed:
                print_success(f"[TASK {current_task}/{num_tasks}] {task.__class__.__name__} completed.")
            elif results is not None and results.failed:
                print_err(f"[TASK {current_task}/{num_tasks}] {task.__class__.__name__} failed on the following hosts: {task._succeded_hosts(results)}")
                exit(1)
            current_task += 1

        # Close connections
        with CONSOLE.status(f"[bold blue] Tasks finished. Closing connections", spinner="point"):
            connections.close()
        print_success(f"Tasks finished. Connections closed. Exiting.")

if __name__ == "__main__":
    try:
        DeployRobots()
    except KeyboardInterrupt:
        print_err("Interrupted by user")
        exit(1)
