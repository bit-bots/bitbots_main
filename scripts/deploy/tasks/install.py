import concurrent.futures

from fabric import Group, GroupResult, Result
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
        try:
            results = connections.run(cmd, hide=hide_output())
            print_debug(f"Internet is available on the following hosts: {self._succeeded_hosts(results)}")
        except GroupException as e:
            print_warn(f"Internet is NOT available and skipping installs on the following hosts: {self._failed_hosts(e.result)}")
            results = e.result
        return results

    def _install_rosdeps(
            self,
            connections: Group
        ) -> GroupResult:
        """
        Install ROS dependencies.
        This is done by simulating a rosdep install to gather install commands (mostly sudo apt install ...).
        Those returned commands are then executed on the remotes.

        This "simulation" or dry-run is necessary, as the rosdep install command does not work correctly if directly invoked with sudo,
        but it calls sudo commands internally, which necessitates user input on the remote.
        The "sudo" functionality provided by fabric is not able to autofill in this case.

        :param connections: The connections to remote servers.
        :return: Results, with success if the Target has an internet connection
        """
        remote_src_path = os.path.join(self._remote_workspace, "src")
        print_debug(f"Gathering rosdep install commands in {remote_src_path}")

        cmd = f"rosdep install --simulate --default-yes --ignore-src --from-paths {remote_src_path}"
        print_debug(f"Calling {cmd}")
        try:
            gather_results = connections.run(cmd, hide=hide_output())
        except GroupException as e:
            print_err(f"Failed to gather rosdeps install commands")
            if not e.result.succeeded:
                return e.result
            gather_results = e.result

        # Only continue with succeeded connections
        connections = get_connections_from_succeeded(gather_results)

        # Define function to multithread install commands on all hosts
        def _install_commands_on_single_host(result: Result) -> Optional[Result]:
            """
            Install all dependencies on a single host from pip and apt.

            :param Result: The result of the simulated rosdep install command.
            :return: The result of the task.
            """
            # Parse rosdep output to get install commands from each succeeded connection
            # The output should look like this:
            # #[apt] Installation commands:
            #   sudo -H apt-get install -y <package1>
            #   sudo -H apt-get install -y <package2>
            #   ...
            connection = result.connection

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
            # pip_command_prefix = ""  # TODO what is it?
            # pip_packages: list[str] = []

            install_result: Optional[Result] = None  # This collects the result of the last run install command, failed if exception occurred
            for install_command in install_commands:
                print_debug(f"Calling {install_command} on {connection.host}")
                if install_command.startswith(apt_command_prefix):
                    # Remove prefix from command, as we collect all apt commands into one
                    apt_packages.append(install_command.replace(apt_command_prefix, "", 1).strip())
                else:
                    print_warn(f"Currently only the apt-get installer is supported. Stumbled over unknown installer in command and skipping it: '{install_command}' on {connection.host}")

            apt_install_command = f"apt-get install -y {' '.join(apt_packages)}"
            print_debug(f"Running apt install command: {apt_install_command} on {connection.host}")
            try:
                install_result = connection.sudo(apt_install_command, hide=hide_output(), password=self._sudo_password, pty=True)
            except Exception as e:
                print_err(f"Failed install command on {connection.host}")
                install_result = Result(connection=connection, exited=1, command=apt_install_command)  # TODO: What exception can we expect here?
            return install_result

        # Collect results from all hosts
        installs_results = GroupResult()

        # Create a ThreadPoolExecutor to run the install commands on all hosts in parallel
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(gather_results.succeeded)) as executor:
            futures = [executor.submit(_install_commands_on_single_host, gather_result) for gather_result in gather_results]

        for future in futures:
            install_result = future.result()
            if install_result is not None:
                installs_results[install_result.connection] = install_result

        return installs_results
