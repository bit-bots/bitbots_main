from typing import Any, Iterable, Optional

import argparse
import ipaddress
import os
import subprocess

import yaml

from fabric import Connection, GroupResult, ThreadingGroup
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich import box


CONSOLE = Console()


def print_err(msg: Any) -> None:
    """Prints an error message in a red box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.ERR_SUCCESS:
        CONSOLE.print(Panel(msg, title="Error", style="bold red", box=box.HEAVY))


def print_warn(msg: Any) -> None:
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


def hide_output() -> bool|str:
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
    with open(_known_targets_path, "r") as f:
        KNOWN_TARGETS: dict[str, dict[str, str]] = yaml.safe_load(f)
except FileNotFoundError:
    print_err(f"Could not find known_targets.yaml in {_known_targets_path}")
    exit(1)

def print_known_targets() -> None:
    table = Table()
    table.add_column("Hostname")
    table.add_column("Robot name")
    table.add_column("IP address")

    known_targets = get_known_targets()
    table.add_row("ALL", "", "")
    for hostname, values in known_targets.items():
        table.add_row(hostname, values.get("robot_name", ""), values.get("ip", ""))
    print_info(f"You can enter the following values as targets:")
    CONSOLE.print(table)
    exit(0)


def get_known_targets() -> dict[str, dict[str, str]]:
    """
    Returns the known targets.

    :return: The known targets.
    """
    return KNOWN_TARGETS


class Target:
    hostname: str
    ip: Optional[ipaddress.IPv4Address | ipaddress.IPv6Address]

    def __init__(self, identifier: str) -> None:
        """
        Target represents a robot to deploy to.
        It can be initialized with a hostname, IP address or a robot name.
        """
        self.hostname, self.ip = self._identify_target(identifier)

    def _identify_target(self, identifier: str) -> tuple[str, Optional[ipaddress.IPv4Address | ipaddress.IPv6Address]]:
        """
        Identifies a target from an identifier.
        The identifier can be a hostname, IP address or a robot name.

        :param identifier: The identifier to identify the target from.
        :return: A tuple containing the hostname and the IP address of the target.
        """
        print_debug(f"Identifying target from identifier: {identifier}")

        identified_target: Optional[str] = None  # The hostname of the identified target

        # Iterate over the known targets
        for hostname, values in KNOWN_TARGETS.items():
            print_debug(f"Checking if {identifier} is {hostname}")

            # Is the identifier a known hostname?
            print_debug(f"Comparing {identifier} with {hostname}")
            if hostname == identifier:
                identified_target = hostname
                break

            # Is the identifier a known robot name?
            print_debug(f"Comparing {identifier} with {values['robot_name']}") if "robot_name" in values else None
            if values.get("robot_name") == identifier:
                identified_target = hostname
                break

            # Is the identifier a known IP address?
            identifier_ip = None
            try:
                print_debug(f"Checking if {identifier} is a IP address")
                identifier_ip = ipaddress.ip_address(identifier)
            except ValueError:
                print_debug(f"Entered target is not a IP-address")
            # We accept every IP address, but if we later find an associated hostname, we use that
            identified_target = str(identifier_ip)

            if "ip" in values:
                try:
                    known_target_ip = ipaddress.ip_address(values["ip"])
                except ValueError:
                    print_warn(f"Invalid IP address ('{values['ip']}') defined for known target: {hostname}")
                    exit(1)

                if identifier_ip is not None and identifier_ip == known_target_ip:
                    identified_target = hostname
                    break

        # If no target was identified, exit
        if identified_target is None:
            print_err(f"Could not find a known target for the given identifier: {identifier}\nChoose from the known targets")
            print_known_targets()
            exit(1)

        print_debug(f"Found {identified_target} as known target")

        identified_ip = None
        if "ip" in KNOWN_TARGETS[identified_target]:
            try:
                identified_ip = ipaddress.ip_address(KNOWN_TARGETS[identified_target]["ip"])
            except ValueError:
                print_err(f"Invalid IP address defined for known target: {identified_target}")
                exit(1)

        return (identified_target, identified_ip)

    def __str__(self) -> str:
        """Returns the target's hostname if available or IP address."""
        return self.hostname if self.hostname is not None else str(self.ip)
    
    def get_connection_identifier(self) -> str:
        """Returns the target's IP address if available or the hostname."""
        return str(self.ip) if self.ip is not None else self.hostname


def _parse_targets(input_targets: str) -> list[Target]:
    """
    Parse target input into usable Targets.

    :param input_targets: The input string of targets as a comma separated string of either hostnames, robot names or IPs. 'ALL' is a valid argument and will be expanded to all known targets.
    :return: List of Targets
    """
    targets: list[Target] = []
    for input_target in input_targets.split(","):
        try:
            target = Target(input_target)
        except ValueError:
            print_err(f"Could not determine hostname or IP from input: '{input_target}'")
            exit(1)
        targets.append(target)
    return targets


def _get_connections_from_targets(
    targets: list[Target],
    user: str,
    connection_timeout: Optional[int] = 10
    ) -> ThreadingGroup:
    """
    Get connections to the given Targets using the 'bitbots' username.

    :param targets: The Targets to connect to
    :param user: The username to connect with
    :param connection_timeout: Timeout for establishing the connection
    :return: The connections
    """
    hosts: list[str] = [target.get_connection_identifier() for target in targets]
    try:
        connections = ThreadingGroup(
            *hosts,
            user=user,
            connect_timeout=connection_timeout
        )
        for connection in connections:
            print_debug(f"Connecting to {connection.host}...")
            connection.open()
            print_debug(f"Connected to {connection.host}...")
    except Exception as e:
        print_err(f"Could not establish all required connections: {hosts}")
        print_debug(e)
        exit(1)
    return connections


def _get_connections_from_all_known_targets(
    user: str,
    connection_timeout: Optional[int] = 10
    ) -> ThreadingGroup:
    """
    Get connections to all known targets using the given username.
    NOTE: This still continues if not all connections could be established.

    :param user: The username to connect with
    :param connection_timeout: Timeout for establishing the connection
    :return: The connections
    """
    # Get hosts from all known targets
    hosts: list[str] = [Target(hostname).get_connection_identifier() for hostname in KNOWN_TARGETS.keys()]

    # Create connections
    connections: list[Connection] = [Connection(host, user=user, connect_timeout=connection_timeout) for host in hosts]

    # Connect to all hosts
    open_connections: list[Connection] = []
    for connection in connections:
        try:
            print_debug(f"Connecting to {connection.host}...")
            connection.open()
            print_debug(f"Connected to {connection.host}...")
        except Exception as e:
            print_err(f"Could not establish connection to {connection.host}. Ignoring this host.")
            print_debug(e)
        open_connections.append(connection)
    if len(open_connections) == 0:
        print_err("Could not establish any connection to the known targets. Exiting...")
        exit(1)
    return ThreadingGroup.from_connections(open_connections)


def get_connections_from_targets(
    input_targets: str,
    user: str,
    connection_timeout: Optional[int] = 10
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
        print_info(f"Connecting to all known Targets: {KNOWN_TARGETS.keys()}")
        return _get_connections_from_all_known_targets(
            user=user,
            connection_timeout=connection_timeout
        )

    return _get_connections_from_targets(
        targets=_parse_targets(input_targets),
        user=user,
        connection_timeout=connection_timeout
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
            print_err(message)
            print_known_targets()
            exit(0)
        else:
            super().error(message)