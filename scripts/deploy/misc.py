import argparse
import ipaddress
import os
import subprocess
import sys
from typing import Any, Iterable, Optional

import yaml
from fabric import Connection, GroupResult, ThreadingGroup
from paramiko import AuthenticationException
from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

CONSOLE = Console()


def print_error(msg: Any) -> None:
    """Prints an error message in a red box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.ERR_SUCCESS:
        CONSOLE.print(Panel(msg, title="Error", style="bold red", box=box.HEAVY))


def print_warning(msg: Any) -> None:
    """Prints a warning message in a yellow box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.WARN:
        CONSOLE.print(Panel(msg, title="Warning", style="yellow", box=box.SQUARE))


def print_success(msg: Any) -> None:
    """Prints a success message in a green box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.ERR_SUCCESS:
        CONSOLE.print(Panel(msg, title="Success", style="green", box=box.SQUARE))


def print_info(msg: Any) -> None:
    """Prints an info message to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.INFO:
        CONSOLE.log(msg, style="")


def print_debug(msg: Any) -> None:
    """Prints a debug message to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.DEBUG:
        CONSOLE.log(msg, style="dim")


def print_bit_bot() -> None:
    """Prints the Bit-Bots logo to the console using cat."""
    path = os.path.join(os.path.dirname(__file__), "bitbot.ans")
    subprocess.run(["cat", path])


class LOGLEVEL:
    ERR_SUCCESS = 0
    WARN = 1
    INFO = 2
    DEBUG = 3
    CURRENT = 2


def be_quiet() -> bool:
    """
    Returns whether to be quiet or not.

    :return: True if the current loglevel is below INFO, False otherwise.
    """
    return LOGLEVEL.CURRENT <= LOGLEVEL.INFO


def hide_output() -> bool | str:
    """
    Returns which output streams to hide.
    stderr is always shown, unless loglevel is below ERR_SUCCESS.
    stdout is shown if loglevel is below DEBUG.

    :return: True if both should be hidden,
        False if both should be shown,
        "stdout" if only stdout should be hidden.
    """
    if LOGLEVEL.CURRENT <= LOGLEVEL.ERR_SUCCESS:
        return True
    elif LOGLEVEL.CURRENT < LOGLEVEL.DEBUG:
        return "stdout"
    else:
        return False


# Read the known targets
_known_targets_path: str = os.path.join(os.path.dirname(__file__), "known_targets.yaml")
try:
    with open(_known_targets_path) as f:
        KNOWN_TARGETS: dict[str, dict[str, str]] = yaml.safe_load(f)
except FileNotFoundError:
    print_error(f"Could not find known_targets.yaml in {_known_targets_path}")
    sys.exit(1)


def print_known_targets() -> None:
    table = Table()
    table.add_column("Hostname")
    table.add_column("Robot name")
    table.add_column("IP address")

    known_targets = get_known_targets()
    table.add_row("ALL", "", "")
    for ip, values in known_targets.items():
        table.add_row(values.get("hostname", ""), values.get("robot_name", ""), ip)
    print_info("You can enter the following values as targets:")
    CONSOLE.print(table)
    sys.exit(0)


def get_known_targets() -> dict[str, dict[str, str]]:
    """
    Returns the known targets.

    :return: The known targets.
    """
    return KNOWN_TARGETS


def _identify_ip(identifier: str) -> str | None:
    """
    Identifies an IP address from an identifier.
    The identifier can be a hostname, IP address or a robot name.

    :param identifier: The identifier to identify the target from.
    :return: IP address of the identified target.
    """
    print_debug(f"Identifying IP address from identifier: '{identifier}'.")

    # Is the identifier an IP address?
    try:
        print_debug(f"Checking if {identifier} is an IP address")
        ip = ipaddress.ip_address(identifier)
        print_debug(f"Identified {ip} as an IP address")
        return str(ip)
    except ValueError:
        print_debug("Entered target is not an IP-address.")

    # It was not an IP address, so we try to find a known target
    for ip, values in KNOWN_TARGETS.items():
        # Is the identifier a known hostname?
        known_hostname = values.get("hostname", None)
        if known_hostname:
            print_debug(f"Comparing {identifier} with {known_hostname}")
            if known_hostname.strip() == identifier.strip():
                print_debug(f"Identified hostname '{known_hostname}' for '{identifier}'. Using its IP {ip}.")
                return str(ipaddress.ip_address(ip))
            else:
                print_debug(f"Hostname '{known_hostname}' does not match identifier '{identifier}'.")

        # Is the identifier a known robot name?
        known_robot_name = values.get("robot_name", None)
        if known_robot_name:
            print_debug(f"Comparing '{identifier}' with '{known_robot_name}'.")
            if known_robot_name.strip() == identifier.strip():
                print_debug(f"Identified robot name '{known_robot_name}' for '{identifier}'. Using its IP {ip}.")
                return str(ipaddress.ip_address(ip))
            else:
                print_debug(f"Robot name '{known_robot_name}' does not match '{identifier}'.")


def _parse_targets(input_targets: str) -> list[str]:
    """
    Parse target input into usable target IP addresses.

    :param input_targets: The input string of targets as a comma separated string of either hostnames, robot names or IPs.
    :return: List of target IP addresses.
    """
    target_ips: list[str] = []
    for input_target in input_targets.split(","):
        try:
            target_ip = _identify_ip(input_target)
        except ValueError:
            print_error(f"Could not determine IP address from input: '{input_target}'")
            sys.exit(1)
        if target_ip is None:
            print_error(f"Could not determine IP address from input:' {input_target}'")
            sys.exit(1)
        target_ips.append(target_ip)
    return target_ips


def _get_connections_from_targets(
    target_ips: list[str], user: str, connection_timeout: Optional[int] = 10
) -> ThreadingGroup:
    """
    Get connections to the given target IP addresses using the given username.

    :param target_ips: The target IP addresses to connect to
    :param user: The username to connect with
    :param connection_timeout: Timeout for establishing the connection
    :return: The connections
    """

    def _concat_exception_args(e: Exception) -> str:
        """Concatenate all arguments of an exception into a string."""
        reason = ""
        for arg in e.args:
            if arg:
                reason += f"{arg} "
        return reason

    connections = ThreadingGroup(*target_ips, user=user, connect_timeout=connection_timeout)
    failures: list[tuple[Connection, str]] = []  # List of tuples of failed connections and their error message
    for connection in connections:
        try:
            print_debug(f"Connecting to {connection.user}@{connection.host}...")
            connection.open()
            print_debug(f"Connected to {connection.user}@{connection.host}...")
            print_debug(f"Getting hostname of {connection.host}...")
            hostname: str = connection.run("hostname", hide=hide_output()).stdout.strip()
            print_debug(f"Got hostname of {connection.host}: {hostname}. Setting is as original hostname.")
            connection.original_host = hostname
        except AuthenticationException as e:
            failures.append(
                (
                    connection,
                    _concat_exception_args(e)
                    + f"Did you add your SSH key to the target? Run '[bold blue]ssh-copy-id {user}@{connection.host}[/bold blue]' manually to do so.",
                )
            )
        except Exception as e:
            failures.append((connection, _concat_exception_args(e)))
    if failures:
        print_error("Could not connect to the following hosts:")
        failure_table = Table(style="bold red", box=box.HEAVY)
        failure_table.add_column("Host")
        failure_table.add_column("Reason")
        [failure_table.add_row(connection.host, reason) for connection, reason in failures]
        CONSOLE.print(failure_table)
        sys.exit(1)
    return connections


def _get_connections_from_all_known_targets(user: str, connection_timeout: Optional[int] = 10) -> ThreadingGroup:
    """
    Get connections to all known targets using the given username.
    NOTE: This still continues if not all connections could be established.

    :param user: The username to connect with
    :param connection_timeout: Timeout for establishing the connection
    :return: The connections
    """
    # Get all known target IP addresses
    target_ips: list[str] = list(KNOWN_TARGETS.keys())

    # Create connections
    connections: list[Connection] = [
        Connection(host, user=user, connect_timeout=connection_timeout) for host in target_ips
    ]

    # Connect to all hosts
    open_connections: list[Connection] = []
    for connection in connections:
        try:
            print_debug(f"Connecting to {connection.host}...")
            connection.open()
            print_debug(f"Connected to {connection.host}...")
        except Exception as e:
            print_error(f"Could not establish connection to {connection.host}. Ignoring this host.")
            print_debug(e)
        open_connections.append(connection)
    if len(open_connections) == 0:
        print_error("Could not establish any connection to the known targets. Exiting...")
        sys.exit(1)
    return ThreadingGroup.from_connections(open_connections)


def get_connections_from_targets(
    input_targets: str, user: str, connection_timeout: Optional[int] = 10
) -> ThreadingGroup:
    """
    First parse the input targets, then get connections to the targets.
    NOTE: If input_targets is 'ALL', all known targets will be used and failed connections will be ignored.

    :param input_targets: The input string of targets as a comma separated string of either hostnames, robot names or IPs. 'ALL' is a valid argument and will be expanded to all known targets.
    :param user: The username to connect with
    :param connection_timeout: Timeout for establishing the connection
    :return: The connections to the targets
    """
    if input_targets == "ALL":
        print_info(f"Connecting to all known targets: {KNOWN_TARGETS.keys()}")
        return _get_connections_from_all_known_targets(user=user, connection_timeout=connection_timeout)

    return _get_connections_from_targets(
        target_ips=_parse_targets(input_targets), user=user, connection_timeout=connection_timeout
    )


def get_connections_from_succeeded(results: GroupResult) -> ThreadingGroup:
    """
    Get connections to the succeeded hosts from the given GroupResult.

    :param results: The GroupResult to get the succeeded hosts from
    :return: The connections
    """
    succeeded_connections: Iterable[Connection] = results.succeeded.keys()
    return ThreadingGroup.from_connections(succeeded_connections)


class ArgumentParserShowTargets(argparse.ArgumentParser):
    """
    This is a normal argparse.ArgumentParser, except, that we intercept the error
    "the following arguments are required" and show the known targets instead
    """

    def error(self, message):
        if "the following arguments are required" in message:
            print_error(message)
            print_known_targets()
            sys.exit(0)
        else:
            super().error(message)
