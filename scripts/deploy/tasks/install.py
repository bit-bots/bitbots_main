import traceback

from fabric import Group, GroupResult
from fabric.exceptions import GroupException

from tasks.abstract_task import AbstractTaskWhichRequiresSudo
from misc import *

class Install(AbstractTaskWhichRequiresSudo):
    def __init__(
            self,
            remote_workspace: str
        ) -> None:
        """
        Task to install and update all dependencies.

        :param remote_workspace: Path to the remote workspace to run rosdep in
        """
        super().__init__()

        self._remote_workspace = remote_workspace

        # TODO: also install pip upgrades
        # TODO: sudo apt update && sudo apt upgrade -y

    def _run(self, connections: Group) -> GroupResult:
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
        results = connections.run(cmd, hide=hide_output())

        if results.succeeded:
            print_debug(f"Internet is available on the following hosts: {self._succeded_hosts(results)}")
        if results.failed:
            print_warn(f"Internet is NOT available and skipping installs on the following hosts: {self._failed_hosts(results)}")
        return results

    def _install_rosdeps(
            self,
            connections: Group
        ) -> GroupResult:
        """
        Install ROS dependencies.
        This is done by simulating a rosdep install to gather install commands (mostly sudo apt install ...).
        Those returned commands are then executed on the remotes.

        This "simulation" or dry-run is necessarry, as the rosdep install command does not work correctly if directly invoked with sudo,
        but it calles sudo commands internally, which necessitates user input on the remote.
        The "sudo" functionality provided by fabric is not able to autofill in this case.

        :param connections: The connections to remote servers.
        :return: Results, with success if the Target has an internet connection
        """
        remote_src_path = os.path.join(self._remote_workspace, "src")
        print_debug(f"Gethering rosdeps install commands in {remote_src_path}")

        cmd = f"rosdep install --simulate --default-yes --ignore-src --from-paths {remote_src_path}"
        print_debug(f"Calling {cmd}")
        try:
            gather_results = connections.run(cmd, hide=hide_output())
        except GroupException as e:
            print_err(f"Failed to gather rosdeps install commands")
            traceback.print_exc()
            return

        if not gather_results.succeeded:
            print_err(f"Failed to gather rosdeps install commands")
            return

        # Parse rosdep output to get install commands from each succeeded connection
        # The output should look like this:
        # #[apt] Installation commands:
        #   sudo -H apt-get install -y <package1>
        #   sudo -H apt-get install -y <package2>
        #   ...
        for connection, result in gather_results.succeeded.items():
            lines = result.stdout.splitlines()
            install_commands = []
            for line in lines:
                if line.startswith("  "):
                    install_commands.append(line.strip())

            if len(install_commands) == 0:
                continue

            print_debug(f"Running install commands from rosdep: {install_commands} on {connection.host}")
            for install_command in install_commands:
                print_debug(f"Calling {install_command} on {connection.host}")
                try:
                    if install_command.startswith("sudo -H"):
                        # Remove sudo from command, as fabric handles sudo internally
                        install_command = install_command.replace("sudo -H", "", 1).strip()
                        connection.sudo(install_command, hide=hide_output(), password=self._sudo_password, pty=True)
                    else:
                        connection.run(install_command, hide=hide_output())
                except GroupException as e:
                    print_err(f"Failed to install rosdeps on {connection.host}")
                    traceback.print_exc()
                    return

        # TODO: parallelize and collect results
        # TODO: Return results