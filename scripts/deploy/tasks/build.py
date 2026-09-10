from fabric import Group, GroupResult

from deploy.misc import get_connections_from_succeeded, print_debug, print_error
from deploy.pixi import pixi_run_command, run_for_connections
from deploy.tasks.abstract_task import AbstractTask


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
        print_debug(f"Cleaning the following packages before building: {self._package}")
        results = run_for_connections(
            connections,
            lambda connection: pixi_run_command(connection, self._remote_workspace, f"clean {self._package}"),
        )
        if results.succeeded:
            print_debug(f"Clean succeeded on the following hosts: {self._succeeded_hosts(results)}")
        if results.failed:
            for connection, result in results.failed.items():
                print_error(f"Clean on {connection.host} failed with the following errors: {result.stderr}")
        return results

    def _build(self, connections: Group) -> GroupResult:
        """
        Build the source code using colcon in the remote workspace.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        print_debug("Building packages")

        results = run_for_connections(
            connections,
            lambda connection: pixi_run_command(connection, self._remote_workspace, f"build {self._package}"),
        )
        if results.succeeded:
            print_debug(f"Build succeeded on the following hosts: {self._succeeded_hosts(results)}")
        if results.failed:
            for connection in results.failed.keys():
                print_error(f"Build on {connection.host} failed!")
        return results
