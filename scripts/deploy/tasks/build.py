from deploy.misc import get_connections_from_succeeded, hide_output, print_debug, print_error
from deploy.tasks.abstract_task import AbstractTask
from fabric import Group, GroupResult
from fabric.exceptions import GroupException


class Build(AbstractTask):
    def __init__(self, remote_workspace: str, package: str = "", pre_clean: bool = False) -> None:
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
        """Clean the source directory before building.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        if self._package:
            print_debug(f"Cleaning the following packages before building: {self._package}")
            cmd_clean = f"colcon clean packages -y --packages-select {self._package}"
        else:
            print_debug("Cleaning ALL packages before building")
            cmd_clean = (
                f"rm -rf {self._remote_workspace}/build {self._remote_workspace}/install {self._remote_workspace}/log"
            )

        print_debug(f"Calling {cmd_clean}")
        try:
            results = connections.run(cmd_clean, hide=hide_output())
            print_debug(f"Clean succeeded on the following hosts: {self._succeeded_hosts(results)}")
        except GroupException as e:
            for connection, result in e.result.failed.items():
                print_error(f"Clean on {connection.host} failed with the following errors: {result.stderr}")
            return e.result
        return results

    def _build(self, connections: Group) -> GroupResult:
        """
        Build the source code using colcon in the remote workspace.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        if self._package:
            print_debug(f"Building the following packages: {self._package}")
            package_option = f"--packages-up-to {self._package}"
        else:
            print_debug("Building ALL packages")
            package_option = ""
        cmd = (
            "sync;"
            f"cd {self._remote_workspace};"
            "source /opt/ros/iron/setup.zsh;"
            "ISOLATED_CPUS=\"$(grep -oP 'isolcpus=\\K([\\d-]+)' /proc/cmdline)\";"  # type: ignore[reportInvalidStringEscapeSequence]
            f"chrt -r 1 taskset -c ${{ISOLATED_CPUS:-0-15}} colcon build --symlink-install {package_option} --continue-on-error || exit 1;"
            "sync;"
        )
        # TODO make output colored
        # TODO: check if only single core?!?

        print_debug(f"Calling {cmd}")
        try:
            results = connections.run(cmd, hide=hide_output())
            print_debug(f"Build succeeded on the following hosts: {self._succeeded_hosts(results)}")
        except GroupException as e:
            for connection in e.result.failed.keys():
                print_error(f"Build on {connection.host} failed!")
            return e.result
        return results
