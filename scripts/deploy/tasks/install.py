from typing import Optional

from fabric import Group, GroupResult

from tasks.abstract_task import AbstractTask
from misc import *

class Install(AbstractTask):
    def __init__(self, remote_workspace: str) -> None:
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
        internet_available_results = self._internet_available_on_target(connections)

        if not internet_available_results.succeeded:
            return internet_available_results
        
        # Some hosts have an internet connection, install rosdeps
        install_results = self._install_rosdeps(get_connections_from_succeeded(internet_available_results))
        return install_results

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
        results = connections.run(cmd)

        if results.succeeded:
            print_debug(f"Internet is available on the following hosts: {self._succeded_hosts(results)}")
        if results.failed:
            print_warn(f"Internet is NOT available and skipping installs on the following hosts: {self._failed_hosts(results)}")
        return results

    def _install_rosdeps(
            self, connections: Group) -> GroupResult:
        """
        Install ROS dependencies using the rosdep tool.

        :param connections: The connections to remote servers.
        :return: Results, with success if the Target has an internet connection
        """
        remote_src_path = os.path.join(self._remote_workspace, "src")

        print_debug(f"Installing rosdeps in {remote_src_path}")
        cmd = f"rosdep install -y --ignore-src --from-paths {remote_src_path}"
        print_debug(f"Calling {cmd}")
        results = connections.run(cmd)

        if results.succeeded:
            print_debug(f"Rosdep install succeded on the following hosts: {self._succeded_hosts(results)}")
        if results.failed:
            print_err(f"Rosdep install failed on the following hosts: {self._failed_hosts(results)}")
        return results
