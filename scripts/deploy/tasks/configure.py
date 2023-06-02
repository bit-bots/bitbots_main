
def configure_game_settings(target: Target) -> None:
    """
    Configure the game settings on the given Target with user input.
    This tries to run the game_settings.py script on the Target.
    
    :param target: The Target to configure the game settings on
    """
    result_game_settings = _execute_on_target(target, f"python3 {target.workspace}/src/bitbots_misc/bitbots_utils/bitbots_utils/game_settings.py", hide=False)
    if not result_game_settings.ok:
        print_err(f"Game settings on {target.hostname} failed")
        exit(result_game_settings.exited)
    print_info(f"Configured game settings on {target.hostname}")


def configure_wifi(target: Target) -> None:
    """
    Configure the wifi on the given Target with user input.
    The user is shown a list of available wifi networks and
    can choose one by answering with the according UUID.
    All other connections are disabled and depriorized.

    :param target: The Target to configure the wifi on
    """
    _execute_on_target(target, "nmcli connection show", hide=False)
    connection_id = Prompt.ask("UUID or name of connection which should be enabled [leave unchanged]", console=CONSOLE)

    if connection_id != "":
        # disable all other connections
        connection_ids = str(_execute_on_target(target, "nmcli --fields UUID,TYPE connection show | grep wifi | awk '{print $1}'").stdout).strip().split("\n")
        for i in connection_ids:
            if not _execute_on_target(target, f'sudo nmcli connection modify {i} connection.autoconnect FALSE').ok:
                print_warn(f"Could not disable connection {i} on {target.hostname}")
            if not _execute_on_target(target, f'sudo nmcli connection modify {i} connection.autoconnect-priority 0').ok:
                print_warn(f"Could not set priority of connection {i} to 0 on {target.hostname}")

        result_enable_connection = _execute_on_target(target, f"sudo nmcli connection up {connection_id}")
        if not result_enable_connection.ok:
            print_err(f"Could not enable connection {connection_id} on {target.hostname}")
            exit(result_enable_connection.exited)
        
        result_set_autoconnect = _execute_on_target(target, f"sudo nmcli connection modify {connection_id} connection.autoconnect TRUE")
        if not result_set_autoconnect.ok:
            print_err(f"Could not enable connection {connection_id} on {target.hostname}")
            exit(result_set_autoconnect.exited)
        result_set_priority = _execute_on_target(target, f"sudo nmcli connection modify {connection_id} connection.autoconnect-priority 100")
        if not result_set_priority.ok:
            print_err(f"Could not set priority of connection {connection_id} to 100 on {target.hostname}")
            exit(result_set_priority.exited)
    print_info(f"Configured wifi on {target.hostname}")


def configure(target: Target) -> None:
    """
    Configure the given Target with user input.
    This includes configuring the game settings and the wifi.

    :param target: The Target to configure
    """
    configure_game_settings(target)
    configure_wifi(target)
