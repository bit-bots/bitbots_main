import argparse
import concurrent.futures
import ipaddress
import os
import subprocess
import sys
from collections.abc import Iterable
from typing import Any

import yaml
from fabric import Connection, GroupResult, ThreadingGroup
from paramiko import AuthenticationException
from rich import box
from rich.console import Console
from rich.console import Group as RichGroup
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
        KNOWN_TARGETS: dict[str, dict[str, str | int]] = yaml.safe_load(f)
except FileNotFoundError:
    print_error(f"Could not find known_targets.yaml in {_known_targets_path}")
    sys.exit(1)


def print_known_targets() -> None:
    table = Table()
    table.add_column("Hostname")
    table.add_column("Robot name")
    table.add_column("Robot number")
    table.add_column("IP address")

    known_targets = get_known_targets()
    table.add_row("ALL", "", "", "")
    for ip, values in known_targets.items():
        table.add_row(values.get("hostname", ""), values.get("robot_name", ""), str(values.get("robot_number", "")), ip)
    print_info("You can enter the following values as targets:")
    CONSOLE.print(table)
    sys.exit(0)


def get_known_targets() -> dict[str, dict[str, str | int]]:
    """
    Returns the known targets.

    :return: The known targets.
    """
    return KNOWN_TARGETS


def _identify_ips(identifier: str) -> list[str]:
    """
    Identifies IP addresses from an identifier.
    The identifier can be a hostname, IP address or a robot name.

    :param identifier: The identifier to identify the target from.
    :return: IP addresses of the identified targets.
    """
    print_debug(f"Identifying IP address from identifier: '{identifier}'.")

    def append_target_ip(target_ips: list[str], ip: str) -> None:
        target_ip = str(ipaddress.ip_address(ip))
        if target_ip not in target_ips:
            target_ips.append(target_ip)

    # Is the identifier an IP address?
    try:
        print_debug(f"Checking if {identifier} is an IP address")
        ip = ipaddress.ip_address(identifier)
        print_debug(f"Identified {ip} as an IP address")
        return [str(ip)]
    except ValueError:
        print_debug("Entered target is not an IP-address.")

    # It was not an IP address, so we try to find a known target
    target_ips: list[str] = []
    for ip, values in KNOWN_TARGETS.items():
        # Is the identifier a known hostname?
        known_hostname = values.get("hostname", None)
        if known_hostname:
            print_debug(f"Comparing {identifier} with {known_hostname}")
            if known_hostname.strip() == identifier.strip():
                print_debug(f"Identified hostname '{known_hostname}' for '{identifier}'. Using its IP {ip}.")
                append_target_ip(target_ips, ip)
            else:
                print_debug(f"Hostname '{known_hostname}' does not match identifier '{identifier}'.")

        # Is the identifier a known robot name?
        known_robot_name = values.get("robot_name", None)
        if known_robot_name:
            print_debug(f"Comparing '{identifier}' with '{known_robot_name}'.")
            if known_robot_name.strip() == identifier.strip():
                print_debug(f"Identified robot name '{known_robot_name}' for '{identifier}'. Using its IP {ip}.")
                append_target_ip(target_ips, ip)
            else:
                print_debug(f"Robot name '{known_robot_name}' does not match '{identifier}'.")

        # Is the identifier a known robot number?
        known_robot_number = values.get("robot_number", None)
        if known_robot_number is not None:
            print_debug(f"Comparing '{identifier}' with robot number '{known_robot_number}'.")
            if str(known_robot_number).strip() == identifier.strip():
                print_debug(f"Identified robot number '{known_robot_number}' for '{identifier}'. Using its IP {ip}.")
                append_target_ip(target_ips, ip)
            else:
                print_debug(f"Robot number '{known_robot_number}' does not match '{identifier}'.")

    return target_ips


def _parse_targets(targets: list[str]) -> list[str]:
    """
    Parse target input into usable target IP addresses.

    :param targets: Targets as a list of strings of either hostnames, robot names, robot numbers or IPs.
    :return: List of target IP addresses.
    """
    return [target_ip for _, target_ips in _parse_target_candidates(targets) for target_ip in target_ips]


def _identify_ips_or_exit(target: str) -> list[str]:
    """
    Identifies IP addresses from an identifier or exits the program if no IP address could be identified

    :param target: The identifier to identify the target from.
    :return: IP addresses of the identified targets.
    """
    try:
        target_ips = _identify_ips(target)
    except ValueError:
        print_error(f"Could not determine IP address from input: '{target}'")
        sys.exit(1)

    if not target_ips:
        print_error(f"Could not determine IP address from input: '{target}'")
        sys.exit(1)

    return target_ips


def _parse_target_candidates(targets: list[str]) -> list[tuple[str, list[str]]]:
    """
    Parse target input into candidate target IP addresses grouped by user input.

    :param targets: Targets as a list of strings of either hostnames, robot names, robot numbers or IPs.
    :return: List of target names and their candidate IP addresses.
    """
    target_candidates: list[tuple[str, list[str]]] = []
    seen_target_ips: set[str] = set()
    for target in targets:
        target_ips = []
        for target_ip in _identify_ips_or_exit(target):
            if target_ip in seen_target_ips:
                continue
            seen_target_ips.add(target_ip)
            target_ips.append(target_ip)

        if target_ips:
            target_candidates.append((target, target_ips))
        else:
            print_debug(f"Target '{target}' only resolved to already requested IPs. Skipping it.")
    return target_candidates


def _get_connections_from_target_candidates(
    target_candidates: list[tuple[str, list[str]]], user: str, connection_timeout: int | None = 10
) -> ThreadingGroup:
    """
    Get connections to the given target IP address candidates using the given username.

    :param target_candidates: The target names and candidate IP addresses to connect to
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

    def _try_connect(target_ip: str) -> tuple[str, Connection | None, str | None]:
        connection = Connection(target_ip, user=user, connect_timeout=connection_timeout)
        try:
            print_debug(f"Connecting to {connection.user}@{connection.host}...")
            connection.open()
            print_debug(f"Connected to {connection.user}@{connection.host}...")
            print_debug(f"Getting hostname of {connection.host}...")
            hostname: str = connection.run("hostname", hide=hide_output()).stdout.strip()
            print_debug(f"Got hostname of {connection.host}: {hostname}. Setting is as original hostname.")
            connection.original_host = hostname
            return target_ip, connection, None
        except AuthenticationException as e:
            reason = (
                _concat_exception_args(e)
                + f"Did you add your SSH key to the target? Run '[bold blue]ssh-copy-id {user}@{connection.host}[/bold blue]' manually to do so."
            )
            return target_ip, None, reason
        except Exception as e:
            return target_ip, None, _concat_exception_args(e)

    open_connections: list[Connection] = []
    connected_target_ips: set[str] = set()
    failed_targets: list[tuple[str, list[tuple[str, str]]]] = []

    for target, target_ips in target_candidates:
        if len(target_ips) > 1:
            print_debug(f"Target '{target}' resolved to multiple candidate IPs: {', '.join(target_ips)}.")

        results_by_ip: dict[str, tuple[Connection | None, str | None]] = {}
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(target_ips)) as executor:
            futures = [executor.submit(_try_connect, target_ip) for target_ip in target_ips]
            for future in concurrent.futures.as_completed(futures):
                target_ip, connection, reason = future.result()
                results_by_ip[target_ip] = (connection, reason)

        successful_target_ips = [target_ip for target_ip in target_ips if results_by_ip[target_ip][0] is not None]
        failures = [
            (target_ip, results_by_ip[target_ip][1] or "")
            for target_ip in target_ips
            if results_by_ip[target_ip][0] is None
        ]

        for target_ip, reason in failures:
            print_debug(f"Could not connect to candidate IP {target_ip} for target '{target}': {reason}")

        if successful_target_ips:
            selected_target_ip = successful_target_ips[0]
            selected_connection = results_by_ip[selected_target_ip][0]
            assert selected_connection is not None
            if selected_target_ip in connected_target_ips:
                print_debug(f"Already connected to {selected_target_ip}. Closing duplicate connection.")
                selected_connection.close()
            else:
                open_connections.append(selected_connection)
                connected_target_ips.add(selected_target_ip)
            for target_ip in successful_target_ips[1:]:
                ignored_connection = results_by_ip[target_ip][0]
                assert ignored_connection is not None
                ignored_connection.close()

            ignored_target_ips = [target_ip for target_ip, _ in failures] + successful_target_ips[1:]
            if ignored_target_ips:
                print_info(
                    f"Connected to target '{target}' via {selected_connection.host}. "
                    f"Ignoring other candidate IPs: {', '.join(ignored_target_ips)}."
                )
            elif len(target_ips) > 1:
                print_debug(f"Connected to target '{target}' via first candidate IP {selected_connection.host}.")
        else:
            failed_targets.append((target, failures))

    if failed_targets:
        failure_table = Table(style="yellow", box=box.SQUARE)
        failure_table.add_column("Target")
        failure_table.add_column("IP address")
        failure_table.add_column("Reason")
        [
            failure_table.add_row(target, target_ip, reason)
            for target, failures in failed_targets
            for target_ip, reason in failures
        ]
        print_warning(RichGroup("Could not connect to the following targets:", failure_table))
    if len(open_connections) == 0:
        print_error("Could not establish any connection to the given targets. Exiting...")
        sys.exit(1)
    return ThreadingGroup.from_connections(open_connections)


def _get_connections_from_all_known_targets(user: str, connection_timeout: int | None = 10) -> ThreadingGroup:
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


def get_connections_from_targets(targets: list[str], user: str, connection_timeout: int | None = 10) -> ThreadingGroup:
    """
    Parses the given targets, then creates connections to the parsed hosts.
    NOTE: If targets is 'ALL', all known targets will be used and failed connections will be ignored.

    :param targets: List of target strings (either hostnames, robot names or IPs, or 'ALL'). If 'ALL' is included, it will be expanded to all known targets.
    :param user: The username to connect with
    :param connection_timeout: Timeout for establishing the connection
    :return: The connections to the targets
    """
    if "ALL" in targets:
        print_info(f"Connecting to targets: {KNOWN_TARGETS.keys()}")
        return _get_connections_from_all_known_targets(user=user, connection_timeout=connection_timeout)

    return _get_connections_from_target_candidates(
        target_candidates=_parse_target_candidates(targets), user=user, connection_timeout=connection_timeout
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
