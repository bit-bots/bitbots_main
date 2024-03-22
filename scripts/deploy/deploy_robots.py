import argparse
import os
import sys
from typing import Optional

from deploy.misc import (
    CONSOLE,
    LOGLEVEL,
    ArgumentParserShowTargets,
    get_connections_from_targets,
    print_bit_bot,
    print_debug,
    print_err,
    print_info,
    print_known_targets,
    print_success,
)
from deploy.tasks import (
    AbstractTask,
    AbstractTaskWhichRequiresSudo,
    Build,
    Configure,
    Install,
    Launch,
    Sync,
)
from rich.prompt import Prompt

# TODO: Install this script as a command line tool


class DeployRobots:
    def __init__(self):
        self._bitbots_main_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        print_info(f"Bit-Bots main path: {self._bitbots_main_path}")
        os.chdir(self._bitbots_main_path)

        # Handle arguments
        self._args = self._parse_arguments()
        LOGLEVEL.CURRENT = LOGLEVEL.CURRENT + self._args.verbose - self._args.quiet

        print_debug(f"Arguments: {self._args}")

        if self._args.show_targets:
            print_known_targets()

        if self._args.print_bit_bot:
            print_bit_bot()

        self._tasks = self._register_tasks()
        self._sudo_password: Optional[str] = self._optionally_ask_for_and_set_sudo_password()

        # Execute tasks on all given targets
        self.run_tasks()

    def _parse_arguments(self) -> argparse.Namespace:
        parser = ArgumentParserShowTargets(
            description="Deploy the Bit-Bots software on a robot. "
            "This script provides 5 tasks: sync, install, configure, build, launch. "
            "By default, it runs all tasks. You can select a subset of tasks by using the corresponding flags. "
            "For example, to only run the sync and build task, use the -sb."
        )

        # Positional arguments
        parser.add_argument(
            "targets",
            type=str,
            help="The targets to deploy to. Multiple targets can be specified separated by commas. 'ALL' can be used to target all known robots.",
        )

        parser.add_argument("--show-targets", action="store_true", help="Show all known targets and exit.")

        # Task arguments
        parser.add_argument(
            "-s",
            "--sync",
            dest="only_sync",
            action="store_true",
            help="Only synchronize (copy) files from you to the target machine.",
        )
        parser.add_argument(
            "-i",
            "--install",
            dest="only_install",
            action="store_true",
            help="Only install ROS dependencies on the targets.",
        )
        parser.add_argument(
            "-c", "--configure", dest="only_configure", action="store_true", help="Only configure the target machines."
        )
        parser.add_argument(
            "-b", "--build", dest="only_build", action="store_true", help="Only build/compile on the target machines."
        )
        parser.add_argument(
            "-l",
            "--launch",
            dest="only_launch",
            action="store_true",
            help="Only launch teamplayer software on the targets.",
        )

        # Optional arguments
        parser.add_argument(
            "-f",
            "--fast",
            action="store_true",
            help="Don't build/compile on the target machines, instead synchronize the complete install directory from the local machine.",
        )
        parser.add_argument("-p", "--package", default="", help="Synchronize and build only the given ROS package.")
        parser.add_argument(
            "--clean",
            action="store_true",
            help="Clean complete workspace (source and install, ...) before syncing and building.",
        )
        parser.add_argument("--clean-src", action="store_true", help="Clean source directory before syncing.")
        parser.add_argument(
            "--clean-build",
            action="store_true",
            help="Clean workspace before building. If --package is given, clean only that package.",
        )
        parser.add_argument("--connection-timeout", default=10, help="Timeout to establish SSH connections in seconds.")
        parser.add_argument(
            "--print-bit-bot", action="store_true", default=False, help="Print our logo at script start."
        )
        parser.add_argument("-v", "--verbose", action="count", default=0, help="More output.")
        parser.add_argument("-q", "--quiet", action="count", default=0, help="Less output.")
        parser.add_argument(
            "-u", "--user", default="bitbots", help="The SSH user to connect to the target machines with"
        )
        parser.add_argument(
            "-w",
            "--workspace",
            default="~/colcon_ws",
            help="Path to the workspace directory to deploy to. Defaults to '~/colcon_ws'",
        )

        args = parser.parse_args()

        if not (args.only_sync or args.only_install or args.only_configure or args.only_build or args.only_launch):
            # By default all tasks are enabled
            args.sync = args.install = args.configure = args.build = args.launch = True
        else:
            # If any of the --only-* arguments is given, disable all tasks and enable only the given ones
            args.sync = args.only_sync
            args.install = args.only_install
            args.configure = args.only_configure
            args.build = args.only_build
            args.launch = args.only_launch

        if args.fast:
            print_info("Fast mode enabled. We will sync the current local colcon workspace and skip the build task.")
            args.sync = True
            args.build = False
            args.clean = True

        if args.clean:
            args.clean_src = True
            args.clean_build = True

        return args

    def _optionally_ask_for_and_set_sudo_password(self) -> Optional[str]:
        """
        Asks the user for the sudo password and returns it.
        Asks only, if tasks that require sudo are enabled.

        :return: The sudo password.
        """
        tasks_with_sudo: list[AbstractTaskWhichRequiresSudo] = [
            task for task in self._tasks if isinstance(task, AbstractTaskWhichRequiresSudo)
        ]
        if tasks_with_sudo:
            sudo_password = Prompt.ask(
                f"Please enter the sudo password for the remote machine (required for {[task.__class__.__name__ for task in tasks_with_sudo]})",
                password=True,
            )
            for task in tasks_with_sudo:
                task.set_sudo_password(sudo_password)

    def _register_tasks(self) -> list[AbstractTask]:
        """
        Register and configure all activated tasks.

        :return: List of tasks
        """
        tasks = []

        if self._args.sync:
            tasks.append(
                Sync(
                    self._bitbots_main_path,
                    self._args.workspace,
                    self._args.package,
                    self._args.clean_src,
                    self._args.fast,
                )
            )

        if self._args.install:
            tasks.append(Install(self._args.workspace))

        if self._args.configure:
            tasks.append(Configure(self._args.workspace))

        if self._args.build:
            tasks.append(
                Build(
                    self._args.workspace,
                    self._args.package,
                    self._args.clean_build,
                )
            )

        if self._args.launch:
            tasks.append(Launch("teamplayer"))

        return tasks

    def run_tasks(self) -> None:
        """
        Main method, that creates connections to all targets and runs all registered tasks in parallel.
        """
        num_tasks = len(self._tasks) + 1  # +1 for establishing connections to the targets
        current_task = 1  # Track current task for status output

        # Get connection
        with CONSOLE.status(
            f"[bold blue][TASK {current_task}/{num_tasks}] Connecting to targets via SSH", spinner="point"
        ):
            connections = get_connections_from_targets(
                self._args.targets, self._args.user, self._args.connection_timeout
            )
        print_success(f"[TASK {current_task}/{num_tasks}] Connected to targets")
        current_task += 1

        # Run tasks
        for task in self._tasks:
            task_prefix = f"[TASK {current_task}/{num_tasks}] {task.__class__.__name__}"
            # Run task
            results = task.run(task_prefix, connections)
            if not results.failed:
                print_success(f"{task_prefix} completed.")
            elif results.failed:
                print_err(f"{task_prefix} failed on the following hosts: {task._failed_hosts(results)}")
                sys.exit(1)
            current_task += 1

        # Close connections
        with CONSOLE.status("[bold blue] Tasks finished. Closing connections", spinner="point"):
            connections.close()
        print_success("Tasks finished. Connections closed. Exiting.")
