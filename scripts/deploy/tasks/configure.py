from fabric import Group, GroupResult, Result
from rich.prompt import Prompt

from tasks.abstract_task import AbstractTask
from misc import *

class Configure(AbstractTask):
    def __init__(
            self,
            remote_workspace:
            str, sudo_password: Optional[str] = ""
        ) -> None:
        """
        Configure the game settings and wifi on the given Targets with user input.

        :param remote_workspace: Path to the remote workspace to run rosdep in
        :param sudo_password: The sudo password of the remote user
        """
        super().__init__()
        self._show_status = False

        self._remote_workspace = remote_workspace
        self._sudo_password = sudo_password

    def _run(self, connections: Group) -> GroupResult:
        """
        Configure the game settings and wifi on the given Targets with user input.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        # First, configure the game settings
        game_settings_results = self._configure_game_settings(connections)
        if not game_settings_results.succeeded:
            return game_settings_results

        # Then, configure the wifi
        wifi_results = self._configure_wifi(
            get_connections_from_succeeded(game_settings_results)
        )
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
            cmd = f"python3 {self._remote_workspace}/src/bitbots_misc/bitbots_utils/bitbots_utils/game_settings.py"
            print_debug(f"Calling {cmd}")
            return connection.run(cmd, hide=False)

        results = GroupResult()

        # Iterate over all connections and configure them
        for connection in connections:
            results[connection] = _configure_single(connection)

        if results.succeeded:
            print_info(f"Game settings configured on the following hosts: {self._succeded_hosts(results)}")
        if results.failed:
            print_err(f"Configuring game setting FAILED on the following hosts: {self._failed_hosts(results)}")
        return results

    def _configure_wifi(self, connections: Group) -> GroupResult:
        """
        Configure the wifi on the given remotes with user input.
        The user is shown a list of available wifi networks and
        can choose one by answering with the according UUID.
        All other connections are disabled and depriorized.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        def _configure_single(
                connection: Connection,
                answered_connection_id: str
            ) -> Optional[Result]:
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
                print_debug(f"Calling {get_ids_cmd} on {connection.host}")
                get_ids_result = connection.run(get_ids_cmd, hide=hide_output())
                if get_ids_result.failed:
                    print_err(f"Could not get connection ids on {connection.host}")
                    return get_ids_result

                connection_ids = str(get_ids_result.stdout).strip().split("\n")
                print_debug(f"Found the following connection ids: {connection_ids} on {connection.host}")

                # Disable autoconnect for all connections and depriorize them except the one we want to use
                for connection_id in connection_ids:
                    if connection_id == answered_connection_id:
                        continue

                    # Disable autoconnect
                    print_debug(f"Disabling autoconnect {connection_id} on {connection.host}")
                    cmd = f"nmcli connection modify {connection_id} connection.autoconnect FALSE"
                    print_debug(f"Calling {cmd} on {connection.host}")
                    disable_autoconnect_result = connection.sudo(cmd, hide=True, password=self._sudo_password)
                    if disable_autoconnect_result.failed:
                        print_err(f"Could not disable autoconnect for connection {connection_id} on {connection.host}")
                        return disable_autoconnect_result

                    # Depriorize connection
                    print_debug(f"Depriorizing connection {connection_id} on {connection.host}")
                    cmd = f"nmcli connection modify {connection_id} connection.autoconnect-priority 0"
                    print_debug(f"Calling {cmd} on {connection.host}")
                    depriorize_result = connection.sudo(cmd, hide=True, password=self._sudo_password)
                    if depriorize_result.failed:
                        print_err(f"Could not depriorize connection {connection_id} on {connection.host}")
                        return depriorize_result

                # Enable the connection we want to use
                print_debug(f"Enabling connection {answered_connection_id} on {connection.host}")
                cmd = f"nmcli connection up {answered_connection_id}"
                print_debug(f"Calling {cmd} on {connection.host}")
                enable_connection_result = connection.sudo(cmd, hide=True, password=self._sudo_password)
                if enable_connection_result.failed:
                    print_err(f"Could not enable connection {answered_connection_id} on {connection.host}")
                    return enable_connection_result

                # Enabling autoconnect for the connection we want to use
                print_debug(f"Enabling autoconnect for connection {answered_connection_id} on {connection.host}")
                cmd = f"nmcli connection modify {answered_connection_id} connection.autoconnect TRUE"
                print_debug(f"Calling {cmd} on {connection.host}")
                enable_autoconnect_result = connection.sudo(cmd, hide=True, password=self._sudo_password)
                if enable_autoconnect_result.failed:
                    print_err(f"Could not enable autoconnect for connection {answered_connection_id} on {connection.host}")
                    return enable_autoconnect_result

                # Set priority for the connection we want to use
                print_debug(f"Setting priority of connection {answered_connection_id} to 100 on {connection.host}")
                cmd = f"nmcli connection modify {answered_connection_id} connection.autoconnect-priority 100"
                print_debug(f"Calling {cmd} on {connection.host}")
                set_priority_result = connection.sudo(cmd, hide=True, password=self._sudo_password)
                if set_priority_result.failed:
                    print_err(f"Could not set priority of connection {answered_connection_id} to 100 on {connection.host}")
                return set_priority_result

        results = GroupResult()

        # Get wifi UUID to enable from user
        # This happens on the first connection in the group
        # UUIDs are the same on all connections
        connection = connections[0]

        # Show available wifi connections
        show_cmd = 'nmcli connection show | grep -E "^(NAME|.*wifi)"'  # Show all wifi connections and table header
        print_debug(f"Calling {show_cmd} on {connection.host}")
        show_result = connection.run(show_cmd, hide=False)
        if show_result.failed:
            print_err(f"Could not show connections on {connection.host}")
            return show_result

        # Ask user for connection to use
        querry = "Enter the UUID of the connection which should be used [Press Enter to leave unchanged]"
        answered_connection_id = Prompt.ask(querry, console=CONSOLE)
        print_debug(f"User answered {answered_connection_id} on {connection.host}")

        # Iterate over all connections and configure them
        with CONSOLE.status("[bold blue] Configuring wifi...") as status:
            for connection in connections:
                results[connection] = _configure_single(connection, answered_connection_id)

        if results.succeeded:
            print_info(f"Wifi configured on the following hosts: {self._succeded_hosts(results)}")
        if results.failed:
            print_err(f"Configuring wifi FAILED on the following hosts: {self._failed_hosts(results)}")
        return results
