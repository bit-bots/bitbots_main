import concurrent.futures
import os
from typing import Optional

from deploy.misc import Connection, get_connections_from_succeeded, hide_output, print_debug, print_error, print_warning
from deploy.tasks import INTERNET_TIMEOUT
from deploy.tasks.abstract_task import AbstractTaskWhichRequiresSudo
from fabric import Group, GroupResult, Result
from fabric.exceptions import GroupException


class Install(AbstractTaskWhichRequiresSudo):
    def __init__(self, remote_workspace: str) -> None:
        """
        Task to install and update all dependencies.

        :param remote_workspace: Path to the remote workspace
        """
        super().__init__()

        self._remote_workspace = remote_workspace

    def _run(self, connections: Group) -> GroupResult:
        """
        Install and update all dependencies, if internet is available.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        internet_available_results = self._internet_available_on_target(connections)
        if not internet_available_results.succeeded:
            return internet_available_results

        # Some hosts have an internet connection, make updates and installs
        apt_upgrade_results = self._apt_upgrade(get_connections_from_succeeded(internet_available_results))
        if not apt_upgrade_results.succeeded:
            return apt_upgrade_results

        basler_install_results = self._install_basler(get_connections_from_succeeded(apt_upgrade_results))
        if not basler_install_results.succeeded:
            return basler_install_results

        rosdep_results = self._install_rosdeps(get_connections_from_succeeded(basler_install_results))
        if not rosdep_results.succeeded:
            return rosdep_results

        pip_upgrade_results = self._pip_upgrade(get_connections_from_succeeded(rosdep_results))
        return pip_upgrade_results

    def _internet_available_on_target(self, connections: Group) -> GroupResult:
        """
        Check if the Target has an internet connection by pinging apt repos.

        :param connections: The connections to remote servers.
        :return: Results, with success if the Target has an internet connection
        """
        apt_mirror = "de.archive.ubuntu.com"
        print_debug("Checking internet connections")

        cmd = f"timeout --foreground {INTERNET_TIMEOUT} curl -sSLI {apt_mirror}"
        print_debug(f"Calling {cmd}")
        try:
            results = connections.run(cmd, hide=hide_output())
            print_debug(f"Internet is available on the following hosts: {self._succeeded_hosts(results)}")
        except GroupException as e:
            print_warning(
                f"Internet is NOT available and skipping installs on the following hosts: {self._failed_hosts(e.result)}"
            )
            results = e.result
        return results

    def _apt_upgrade(self, connections: Group) -> GroupResult:
        """
        Upgrade all apt packages on the target.
        Runs apt update and apt upgrade -y.

        :param connections: The connections to remote servers.
        :return: Results, with success if the upgrade succeeded on the target
        """
        print_debug("Updating apt")

        cmd = "apt update"
        print_debug(f"Calling {cmd}")
        try:
            update_results = connections.sudo(cmd, hide=hide_output(), password=self._sudo_password)
            print_debug(f"Updated apt on the following hosts: {self._succeeded_hosts(update_results)}")
        except GroupException as e:
            print_error(f"Failed to update apt on the following hosts: {self._failed_hosts(e.result)}")
            update_results = e.result

        print_debug("Upgrading apt packages")

        cmd = "apt upgrade -y"
        print_debug(f"Calling {cmd}")
        try:
            upgrade_results = connections.sudo(cmd, hide=hide_output(), password=self._sudo_password)
            print_debug(f"Upgraded apt packages on the following hosts: {self._succeeded_hosts(upgrade_results)}")
        except GroupException as e:
            print_error(f"Failed to upgrade apt packages on the following hosts: {self._failed_hosts(e.result)}")
            upgrade_results = e.result
        return update_results

    def _install_basler(self, connections: Group) -> GroupResult:
        """
        Installs the basler camera drivers on the targets.

        :param connections: The connections to remote servers.
        :return: Results, with success if the install succeeded on the target
        """
        print_debug("Installing basler drivers")

        cmd = f"{self._remote_workspace}/src/scripts/make_basler.sh --ci"
        print_debug(f"Calling {cmd}")
        try:
            install_results = connections.sudo(cmd, hide=hide_output(), password=self._sudo_password)
            print_debug(f"Installed basler drivers on the following hosts: {self._succeeded_hosts(install_results)}")
        except GroupException as e:
            print_error(f"Failed to install basler drivers on the following hosts: {self._failed_hosts(e.result)}")
            install_results = e.result
        return install_results

    def _install_rosdeps(self, connections: Group) -> GroupResult:
        """
        Install ROS dependencies.
        This is done by simulating a rosdep install to gather install commands (mostly sudo apt install ...).
        Those returned commands are then executed on the remotes.

        This "simulation" or dry-run is necessary, as the rosdep install command does not work correctly if directly invoked with sudo,
        but it calls sudo commands internally, which necessitates user input on the remote.
        The "sudo" functionality provided by fabric is not able to autofill in this case.

        :param connections: The connections to remote servers.
        :return: Results, with success if the install commands succeeded on the target
        """
        remote_src_path = os.path.join(self._remote_workspace, "src")
        print_debug(f"Gathering rosdep install commands in {remote_src_path}")

        cmd = f"rosdep update && rosdep install --rosdistro jazzy --simulate --default-yes --ignore-src --from-paths {remote_src_path}"
        print_debug(f"Calling {cmd}")
        try:
            gather_results = connections.run(cmd, hide=hide_output())
        except GroupException as e:
            print_error("Failed to gather rosdeps install commands")
            if not e.result.succeeded:
                return e.result
            gather_results = e.result

        # Only continue with succeeded connections
        connections = get_connections_from_succeeded(gather_results)

        # Define function to multithread install commands on all hosts
        def _install_commands_on_single_host(connection: Connection, result: Result) -> Optional[Result]:
            """
            Install all dependencies on a single host from pip and apt.

            :param connection: The connection to the remote server.
            :param Result: The result of the simulated rosdep install command.
            :return: The result of the task.
            """
            # Parse rosdep output to get install commands from each succeeded connection
            # The output should look like this:
            # #[apt] Installation commands:
            #   sudo -H apt-get install -y <package1>
            #   sudo -H apt-get install -y <package2>
            #   ...
            lines = result.stdout.splitlines()
            install_commands = []
            for line in lines:
                if line.startswith("  "):
                    install_commands.append(line.strip())

            if len(install_commands) == 0:
                print_debug(f"Nothing to install for {connection.host}")
                return Result(connection=connection, exited=0)  # Successful exit code

            # Define command prefixes to search for
            apt_command_prefix = "sudo -H apt-get install -y "
            apt_packages: list[str] = []

            install_result: Optional[Result] = (
                None  # This collects the result of the last run install command, failed if exception occurred
            )
            for install_command in install_commands:
                if install_command.startswith(apt_command_prefix):
                    # Remove prefix from command, as we collect all apt commands into one
                    apt_packages.append(install_command.replace(apt_command_prefix, "", 1).strip())
                else:
                    print_warning(
                        f"Currently only the apt-get installer is supported. Stumbled over unknown installer in command and skipping it: '{install_command}' on {connection.host}"
                    )

            apt_install_command = f"apt-get install -y {' '.join(apt_packages)}"
            print_debug(f"Running apt install command: {apt_install_command} on {connection.host}")
            try:
                install_result = connection.sudo(apt_install_command, hide=hide_output(), password=self._sudo_password)
            except Exception:  # TODO: What exception can we expect here?
                print_error(f"Failed install command on {connection.host}")
                install_result = Result(connection=connection, exited=1, command=apt_install_command)
            return install_result

        # Collect results from all hosts
        installs_results = GroupResult()

        # Create a ThreadPoolExecutor to run the install commands on all hosts in parallel
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(gather_results.succeeded)) as executor:
            futures = [
                executor.submit(_install_commands_on_single_host, gather_connection, gather_result)
                for gather_connection, gather_result in gather_results.items()
            ]

        for future in futures:
            install_result = future.result()
            if install_result is not None:
                installs_results[install_result.connection] = install_result

        # Re-Bifurcate results into succeeded and failed results
        # This is necessary, as the GroupResult does this prematurely, before we could collect the results from all hosts
        for key, value in installs_results.items():
            if value.exited != 0:
                installs_results.failed[key] = value
            else:
                installs_results.succeeded[key] = value

        return installs_results

    def _pip_upgrade(self, connections: Group) -> GroupResult:
        """
        Install and upgrade all pip robot requirements on the target.

        :param connections: The connections to remote servers.
        :return: Results, with success if the upgrade succeeded on the target
        """
        print_debug("Upgrading pip packages")

        cmd = f"pip3 install --user --upgrade --break-system-packages -r {self._remote_workspace}/src/requirements/robot.txt"
        print_debug(f"Calling {cmd}")
        try:
            upgrade_results = connections.run(cmd, hide=hide_output())
            print_debug(f"Upgraded pip packages on the following hosts: {self._succeeded_hosts(upgrade_results)}")
        except GroupException as e:
            print_error(f"Failed to upgrade pip packages on the following hosts: {self._failed_hosts(e.result)}")
            upgrade_results = e.result
        return upgrade_results
