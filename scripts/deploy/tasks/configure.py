import concurrent.futures

from deploy.misc import (
    CONSOLE,
    Connection,
    get_connections_from_succeeded,
    hide_output,
    print_debug,
    print_error,
    print_info,
)
from deploy.tasks.abstract_task import AbstractTaskWhichRequiresSudo
from fabric import Group, GroupResult, Result
from rich.prompt import Prompt


class Configure(AbstractTaskWhichRequiresSudo):
    def __init__(self, remote_workspace: str) -> None:
        """
        Configure the game settings and WiFi on the given Targets with user input.

        :param remote_workspace: Path to the remote workspace to run rosdep in
        """
        super().__init__()
        self._show_status = False

        self._remote_workspace = remote_workspace

    def _run(self, connections: Group) -> GroupResult:
        """
        Configure the game settings and WiFi on the given Targets with user input.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        # First, configure the game settings
        game_settings_results = self._configure_game_settings(connections)
        if not game_settings_results.succeeded:
            return game_settings_results

        # Then, configure the WiFi
        wifi_results = self._configure_wifi(get_connections_from_succeeded(game_settings_results))
        return wifi_results

    def _configure_game_settings(self, connections: Group) -> GroupResult:
        """
        Configure the game settings on the given remotes with user input.
        This tries to run the game_settings.py script on the remotes.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """

        def _configure_single(connection: Connection) -> Result:
            """
            Configure the game settings on a single Target with user input.

            :param connection: The connection to the remote server.
            :return: The result of the task.
            """
            print_info(f"Configuring game settings on {connection.host}...")
            cmd = f"python3 {self._remote_workspace}/src/bitbots_misc/bitbots_parameter_blackboard/bitbots_parameter_blackboard/game_settings.py"

            print_debug(f"Calling '{cmd}'")
            results = connection.run(
                cmd, hide=False, pty=True
            )  # TODO: Does this need a try block? Which exceptions can occur?
            return results

        # Collect results of the group
        results = GroupResult()

        # Iterate over all connections and configure them
        # NOTE: This is not parallelized because for each connection, user input is required
        for connection in connections:
            results[connection] = _configure_single(connection)

        if results.succeeded:
            print_info(f"Game settings configured on the following hosts: {self._succeeded_hosts(results)}")
        if results.failed:
            print_error(f"Configuring game setting FAILED on the following hosts: {self._failed_hosts(results)}")
        return results

    def _configure_wifi(self, connections: Group) -> GroupResult:
        """
        Configure the wifi on the given remotes with user input.
        The user is shown a list of available wifi networks and
        can choose one by answering with the according UUID.
        All other connections are disabled and de-prioritized.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """

        def _configure_single(connection: Connection, answered_connection_id: str) -> Result | None:
            """
            Configure the wifi on a single remote with user input.

            :param connection: The connection to the remote server.
            :param answered_connection_id: The UUID of the wifi network to use.
            :return: The result of the task.
            """
            print_info(f"Configuring wifi on {connection.host}...")

            # Only change connection if user input is not empty
            if answered_connection_id != "":
                # Disable all other connections
                print_debug(f"Disabling all other connections on {connection.host}")
                # Get all wifi connection ids
                get_ids_cmd = "nmcli --fields UUID,TYPE connection show | grep wifi | awk '{print $1}'"
                print_debug(f"Calling '{get_ids_cmd}' on: {connection.host}")
                get_ids_result = connection.run(
                    get_ids_cmd, hide=hide_output()
                )  # TODO: Does this need a try block? Which exceptions can occur?
                if get_ids_result.failed:
                    print_error(f"Could not get connection ids on {connection.host}")
                    return get_ids_result

                connection_ids = str(get_ids_result.stdout).strip().split("\n")
                print_debug(f"Found the following connection ids: {connection_ids} on {connection.host}")

                # Disable autoconnect for all connections and de-prioritize them except the one we want to use
                for connection_id in connection_ids:
                    if connection_id == answered_connection_id:
                        continue

                    # Disable autoconnect
                    print_debug(f"Disabling autoconnect {connection_id} on {connection.host}")
                    cmd = f"nmcli connection modify {connection_id} connection.autoconnect FALSE"
                    print_debug(f"Calling '{cmd}' on: {connection.host}")
                    disable_autoconnect_result = connection.sudo(
                        cmd, hide=True, password=self._sudo_password
                    )  # TODO: Does this need a try block? Which exceptions can occur?
                    if disable_autoconnect_result.failed:
                        print_error(
                            f"Could not disable autoconnect for connection {connection_id} on {connection.host}"
                        )
                        return disable_autoconnect_result

                    # De-prioritize connection
                    print_debug(f"De-prioritizing connection {connection_id} on {connection.host}")
                    cmd = f"nmcli connection modify {connection_id} connection.autoconnect-priority 0"
                    print_debug(f"Calling '{cmd}' on: {connection.host}")
                    de_prioritize_result = connection.sudo(
                        cmd, hide=True, password=self._sudo_password
                    )  # TODO: Does this need a try block? Which exceptions can occur?
                    if de_prioritize_result.failed:
                        print_error(f"Could not de-prioritize connection {connection_id} on {connection.host}")
                        return de_prioritize_result

                # Enable the connection we want to use
                print_debug(f"Enabling connection {answered_connection_id} on {connection.host}")
                cmd = f"nmcli connection up {answered_connection_id}"
                print_debug(f"Calling '{cmd}' on: {connection.host}")
                enable_connection_result = connection.sudo(
                    cmd, hide=True, password=self._sudo_password
                )  # TODO: Does this need a try block? Which exceptions can occur?
                if enable_connection_result.failed:
                    print_error(f"Could not enable connection {answered_connection_id} on {connection.host}")
                    return enable_connection_result

                # Enabling autoconnect for the connection we want to use
                print_debug(f"Enabling autoconnect for connection {answered_connection_id} on {connection.host}")
                cmd = f"nmcli connection modify {answered_connection_id} connection.autoconnect TRUE"
                print_debug(f"Calling '{cmd}' on: {connection.host}")
                enable_autoconnect_result = connection.sudo(
                    cmd, hide=True, password=self._sudo_password
                )  # TODO: Does this need a try block? Which exceptions can occur?
                if enable_autoconnect_result.failed:
                    print_error(
                        f"Could not enable autoconnect for connection {answered_connection_id} on {connection.host}"
                    )
                    return enable_autoconnect_result

                # Set priority for the connection we want to use
                print_debug(f"Setting priority of connection {answered_connection_id} to 100 on {connection.host}")
                cmd = f"nmcli connection modify {answered_connection_id} connection.autoconnect-priority 100"
                print_debug(f"Calling '{cmd}' on: {connection.host}")
                set_priority_result = connection.sudo(
                    cmd, hide=True, password=self._sudo_password
                )  # TODO: Does this need a try block? Which exceptions can occur?
                if set_priority_result.failed:
                    print_error(
                        f"Could not set priority of connection {answered_connection_id} to 100 on {connection.host}"
                    )
                return set_priority_result

        # Get wifi UUID to enable from user
        # This happens on the first connection in the group
        # UUIDs are the same on all connections
        connection = connections[0]

        # Show available wifi connections
        show_cmd = 'nmcli connection show | grep -E "^(NAME|.*wifi)"'  # Show all wifi connections and table header
        print_debug(f"Calling '{show_cmd}' on: {connection.host}")
        show_result = connection.run(
            show_cmd, hide=False
        )  # TODO: Does this need a try block? Which exceptions can occur?
        if show_result.failed:
            print_error(f"Could not show connections on {connection.host}")
            return show_result

        # Ask user for connection to use
        query = "Enter the UUID of the connection which should be used [Press Enter to leave unchanged]"
        answered_connection_id = Prompt.ask(query, console=CONSOLE)

        if answered_connection_id:  # User entered a connection id, we need to configure wifi
            print_debug(f"User answered {answered_connection_id} on {connection.host}")

            # User input is done, we can show a status message again
            with CONSOLE.status("[bold blue] Configuring wifi...", spinner="point"):
                # Collect results
                results = GroupResult()

                # Configure wifi on all connections in parallel
                # Create a ThreadPoolExecutor to run the _configure_single function in parallel
                with concurrent.futures.ThreadPoolExecutor(max_workers=len(connections)) as executor:
                    # Create a future for each connection
                    futures = [
                        executor.submit(_configure_single, connection, answered_connection_id)
                        for connection in connections
                    ]

                # Wait for all futures to complete
                for future in futures:
                    result: Result = future.result()  # type: ignore
                    results[result.connection] = result

                if results.succeeded:
                    print_info(f"Wifi configured on the following hosts: {self._succeeded_hosts(results)}")
                if results.failed:
                    print_error(f"Configuring wifi FAILED on the following hosts: {self._failed_hosts(results)}")
            return results

        else:  # User did not answer, we do not change anything
            print_debug("User did not answer. Leaving wifi unchanged.")
        return GroupResult()
