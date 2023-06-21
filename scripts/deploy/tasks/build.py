from fabric import Group, GroupResult

from tasks.abstract_task import AbstractTask
from misc import *


class Build(AbstractTask):
    def __init__(
            self,
            remote_workspace: str,
            package: str = '',
            pre_clean: bool = False
        ) -> None:
        """
        Task to build using colcon in the remote workspace.

        :param remote_workspace: Path to the remote workspace to run colcon in
        :param package: Limit compilation to file from this package, if empty, all packages will be build
        :param pre_clean: Whether to clean the source directory before building
        """
        super().__init__()

        self._remote_workspace = remote_workspace
        self._package = package
        self._pre_clean = pre_clean

    def _run(self, connections: Group) -> GroupResult:
        """
        Compile source code using colcon in the remote workspace.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        if self._pre_clean:
            clean_results = self._clean(connections)
            if not clean_results.succeeded:
                return clean_results
            connections = get_connections_from_succeeded(clean_results)

        build_results = self._build(connections)
        return build_results


    def _clean(self, connections: Group) -> GroupResult:
        if self._package:
            print_debug(f"Cleaning the following packages before building: {self._package}")
            cmd_clean = f"colcon clean packages -y --packages-select {self._package}"
        else:
            print_debug(f"Cleaning ALL packages before building")
            cmd_clean = 'colcon clean workspace -y'

        print_debug(f"Calling {cmd_clean}")
        results = connections.run(cmd_clean, hide=hide_output())

        if results.succeeded:
            print_debug(f"Clean succeeded on the following hosts: {self._succeeded_hosts(results)}")
        if results.failed:
            for connection, result in results.failed.items():
                print_err(f"Clean on {connection.host} failed with the following errors: {result.stderr}")
        return results

    def _build(self, connections: Group) -> GroupResult:
        if self._package:
            print_debug(f"Building the following packages: {self._package}")
            package_option = f"--packages-up-to {self._package}"
        else:
            print_debug(f"Building ALL packages")
            package_option = ""
        cmd = (
            "sync;"
            f"cd {self._remote_workspace};"
            "source /opt/ros/rolling/setup.zsh;"
            "ISOLATED_CPUS=\"$(grep -oP 'isolcpus=\K([\d,]+)' /proc/cmdline)\";"  # type: ignore[reportInvalidStringEscapeSequence]
            f"chrt -r 1 taskset -c ${{ISOLATED_CPUS:-0-15}} colcon build --symlink-install {package_option} --continue-on-error || exit 1;"
            "sync;"
        )
        # TODO make output colored

        print_debug(f"Calling {cmd}")
        results = connections.run(cmd, hide=hide_output())

        if results.succeeded:
            print_debug(f"Build succeeded on the following hosts: {self._succeeded_hosts(results)}")
        if results.failed:
            for connection, result in results.failed.items():
                print_err(f"Build on {connection.host} failed with the following errors: {result.stderr}")
        return results
