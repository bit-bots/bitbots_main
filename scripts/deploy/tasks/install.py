from typing import Iterable

from fabric import Connection, Group, GroupResult, Result

from tasks.abstract_task import AbstractTask
from misc import *

class Install(AbstractTask):
    def __init__(
            self,
            remote_workspace: str,
        ) -> None:
        """
        Task to install and update all dependencies.

        :param remote_workspace: Path to the remote workspace to run rosdep in
        """
        super().__init__()

        self._remote_workspace = remote_workspace

        # TODO: also install pip upgrades
        # TODO: sudo apt update && sudo apt upgrade -y

    def run(self, connections: Group) -> GroupResult:
        """
        Install and update all dependencies, if internet is available.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        internet_avalible_results = self._internet_available_on_target(connections)

        print_warn(f"Internet is NOT available and skipping installs on the following hosts: {self._failed_hosts(internet_avalible_results)}")
        print_debug(f"Internet is available on the following hosts: {self._succeded_hosts(internet_avalible_results)}")

        hosts_with_internet = internet_avalible_results.succeeded.keys()

        return self._install_rosdeps(hosts_with_internet)

    def _internet_available_on_target(self, connections: Group) -> GroupResult:
        """
        Check if the Target has an internet connection by pinging apt repos.
        
        :param connections: The connections to remote servers.
        :return: Results, with success if the Target has an internet connection
        """
        apt_mirror = "de.archive.ubuntu.com"
        print_debug(f"Checking internet connections")

        cmd = f"timeout --foreground 0.5 curl -sSLI {apt_mirror}"
        print_debug(f"Calling {cmd}")
        return connections.run(cmd)

    def _install_rosdeps(
            self,
            connections: Iterable[Connection],
        ) -> list[Result]:
        """
        Install ROS dependencies using the rosdep tool.

        :param connections: The connections to remote servers.
        :return: Results, with success if the Target has an internet connection
        """
        # TODO find solution to use Group and Group results instead to run parallel
        remote_src_path = os.path.join(self._remote_workspace, "src")
        print_debug(f"Installing rosdeps in {remote_src_path}")

        cmd = f"rosdep install -y --ignore-src --from-paths {remote_src_path}"

        results: list[Result] = []
        for connection in connections:
            print_debug(f"Calling {cmd} on host {connection.host}")
            results.append(connection.run(cmd))
        return results
