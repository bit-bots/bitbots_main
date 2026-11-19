import argparse
import concurrent.futures
import ipaddress
import os
import re
import subprocess
import sys
from collections.abc import Iterable
from dataclasses import dataclass
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


def check_ssh_configuration(user: str, host: str) -> None:
    """Validate the SSH configuration before rsync invokes SSH as a subprocess."""
    result = subprocess.run(
        ["ssh", "-G", "-o", "StrictHostKeyChecking=no", f"{user}@{host}"],
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        return

    error = result.stderr.strip()
    permissions_error = re.search(r"Bad owner or permissions on (.+)", error)
    if permissions_error:
        config_path = permissions_error.group(1)
        print_error(
            f"OpenSSH rejected the configuration file '{config_path}' because its owner or permissions are unsafe. "
            f"Ensure that you own the file and remove write permissions for group and others, for example with "
            f"'chmod go-w {config_path}'."
        )
    else:
        print_error(f"OpenSSH could not read the SSH configuration:\n{error}")

    result.check_returncode()


@dataclass(frozen=True)
class KnownRobot:
    """A robot from known_targets.yaml."""

    name: str
    number: int
    aliases: tuple[str, ...]
    ips: tuple[str, ...]

    def is_called(self, identifier: str) -> bool:
        """
        Whether the identifier is this robot's number, name or one of its aliases.
        Names and aliases are compared case-insensitively.
        """
        identifier = identifier.strip().casefold()
        return identifier == str(self.number) or any(
            identifier == name.casefold() for name in (self.name, *self.aliases)
        )


@dataclass(frozen=True)
class TargetCandidate:
    """A target given by the user, with the IP addresses that may reach it."""

    target: str
    ips: list[str]
    robot: KnownRobot | None

    @property
    def label(self) -> str:
        """The robot name if the target is a known robot, otherwise the user input."""
        return self.robot.name if self.robot is not None else self.target


_KNOWN_ROBOT_KEYS = {"name", "number", "aliases", "ips"}
_ALL_TARGETS = "ALL"


def _is_ip_address(value: str) -> bool:
    """Whether the value is interpreted as a literal IP address when given as a target."""
    try:
        ipaddress.ip_address(value.strip())
    except ValueError:
        return False
    return True


def _parse_known_robots(data: Any) -> list[KnownRobot]:
    """
    Parses and validates the content of known_targets.yaml.

    :param data: The loaded YAML content.
    :return: The known robots.
    :raises ValueError: If the content is malformed or ambiguous.
    """
    if not isinstance(data, list):
        raise ValueError("Expected a list of robots.")

    robots: list[KnownRobot] = []
    # Maps each casefolded name and alias to the index of the entry that owns it.
    # The index is used instead of the name, so that two entries with the same name are detected.
    seen_names: dict[str, int] = {}
    seen_numbers: dict[int, str] = {}
    seen_ips: dict[str, str] = {}
    for index, entry in enumerate(data):
        if not isinstance(entry, dict):
            raise ValueError(f"Entry {index} is not a mapping.")
        unknown_keys = set(entry) - _KNOWN_ROBOT_KEYS
        if unknown_keys:
            raise ValueError(f"Entry {index} has unknown keys: {', '.join(sorted(map(str, unknown_keys)))}.")

        name = entry.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"Entry {index} needs a non-empty 'name'.")
        name = name.strip()

        number = entry.get("number")
        if not isinstance(number, int) or isinstance(number, bool):
            raise ValueError(f"Robot '{name}' needs an integer 'number'.")

        aliases = entry.get("aliases", [])
        if not isinstance(aliases, list) or not all(isinstance(alias, str) and alias.strip() for alias in aliases):
            raise ValueError(f"Robot '{name}' has 'aliases' that are not a list of non-empty strings.")
        aliases = [alias.strip() for alias in aliases]

        raw_ips = entry.get("ips")
        if not isinstance(raw_ips, list) or not raw_ips:
            raise ValueError(f"Robot '{name}' needs a non-empty list of 'ips'.")
        try:
            ips = [str(ipaddress.ip_address(str(ip))) for ip in raw_ips]
        except ValueError as e:
            raise ValueError(f"Robot '{name}' has an invalid IP address: {e}") from e

        for robot_name in (name, *aliases):
            if robot_name.removeprefix("-").isdigit():
                raise ValueError(f"Name '{robot_name}' of robot '{name}' would be ambiguous with robot numbers.")
            if robot_name.casefold() == _ALL_TARGETS.casefold():
                raise ValueError(f"Name '{robot_name}' of robot '{name}' is reserved for targeting all robots.")
            if _is_ip_address(robot_name):
                raise ValueError(f"Name '{robot_name}' of robot '{name}' would be ambiguous with IP addresses.")
            owner = seen_names.setdefault(robot_name.casefold(), index)
            if owner != index:
                raise ValueError(
                    f"Name '{robot_name}' of robot '{name}' is already used by robot '{robots[owner].name}'."
                )
        if number in seen_numbers:
            raise ValueError(f"Number {number} of robot '{name}' is already used by robot '{seen_numbers[number]}'.")
        seen_numbers[number] = name
        for ip in ips:
            if ip in seen_ips:
                raise ValueError(f"IP address {ip} of robot '{name}' is already used by robot '{seen_ips[ip]}'.")
            seen_ips[ip] = name

        robots.append(KnownRobot(name=name, number=number, aliases=tuple(aliases), ips=tuple(ips)))
    return robots


def _load_known_robots(path: str) -> list[KnownRobot]:
    """Loads the known robots or exits the program if the file is missing or malformed."""
    try:
        with open(path) as f:
            return _parse_known_robots(yaml.safe_load(f))
    except FileNotFoundError:
        print_error(f"Could not find known_targets.yaml in {path}")
    except (yaml.YAMLError, ValueError) as e:
        print_error(f"Invalid known targets in {path}: {e}")
    sys.exit(1)


KNOWN_ROBOTS: list[KnownRobot] = _load_known_robots(os.path.join(os.path.dirname(__file__), "known_targets.yaml"))


def print_known_targets() -> None:
    show_aliases = any(robot.aliases for robot in KNOWN_ROBOTS)
    table = Table()
    table.add_column("Number")
    table.add_column("Name")
    if show_aliases:
        table.add_column("Aliases")
    table.add_column("IP addresses")

    def add_row(number: str, name: str, aliases: str, ips: str) -> None:
        table.add_row(number, name, *([aliases] if show_aliases else []), ips)

    add_row("", _ALL_TARGETS, "", "")
    for robot in KNOWN_ROBOTS:
        add_row(str(robot.number), robot.name, ", ".join(robot.aliases), ", ".join(robot.ips))
    print_info("You can enter the following values as targets (names are case-insensitive), or any other IP address:")
    CONSOLE.print(table)
    sys.exit(0)


def _identify_target(identifier: str) -> TargetCandidate | None:
    """
    Identifies a target from an identifier.
    The identifier can be an IP address, or the number, name or an alias of a known robot.
    A known robot resolves to all of its IP addresses, an IP address only to itself.

    :param identifier: The identifier to identify the target from.
    :return: The identified target, or None if the identifier is unknown.
    """
    try:
        ip = str(ipaddress.ip_address(identifier.strip()))
    except ValueError:
        print_debug(f"'{identifier}' is not an IP address. Looking for a known robot.")
    else:
        robot = next((robot for robot in KNOWN_ROBOTS if ip in robot.ips), None)
        print_debug(f"Identified IP address {ip}" + (f" of robot '{robot.name}'." if robot else "."))
        return TargetCandidate(target=identifier, ips=[ip], robot=robot)

    for robot in KNOWN_ROBOTS:
        if robot.is_called(identifier):
            print_debug(f"Identified robot '{robot.name}' for '{identifier}' with IPs {', '.join(robot.ips)}.")
            return TargetCandidate(target=identifier, ips=list(robot.ips), robot=robot)
    return None


def _parse_targets(targets: list[str]) -> list[str]:
    """
    Parse target input into usable target IP addresses.

    :param targets: Targets as a list of strings of either robot numbers, names, aliases or IPs.
    :return: List of target IP addresses.
    """
    return [target_ip for candidate in _parse_target_candidates(targets) for target_ip in candidate.ips]


def _parse_target_candidates(targets: list[str]) -> list[TargetCandidate]:
    """
    Parse target input into candidate target IP addresses grouped by user input.
    Targets that refer to an already given robot or IP address are skipped.

    :param targets: Targets as a list of strings of either robot numbers, names, aliases or IPs.
    :return: The targets and their candidate IP addresses.
    """
    target_candidates: list[TargetCandidate] = []
    given_targets: set[str] = set()
    for target in targets:
        candidate = _identify_target(target)
        if candidate is None:
            print_error(f"Could not determine IP address from input: '{target}'")
            sys.exit(1)

        key = candidate.robot.name if candidate.robot is not None else candidate.ips[0]
        if key in given_targets:
            print_info(f"Target '{target}' refers to the already given target '{candidate.label}'. Skipping it.")
            continue

        given_targets.add(key)
        target_candidates.append(candidate)
    return target_candidates


def _get_connections_from_target_candidates(
    target_candidates: list[TargetCandidate], user: str, connection_timeout: int | None = 10
) -> ThreadingGroup:
    """
    Get connections to the given target IP address candidates using the given username.
    For each target, the first reachable candidate IP address is used.
    Each connection's original_host is set to the robot name, or to the remote hostname for unknown targets.

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

    def _try_connect(target_ip: str, robot: KnownRobot | None) -> tuple[str, Connection | None, str | None]:
        connection = Connection(target_ip, user=user, connect_timeout=connection_timeout)
        try:
            print_debug(f"Connecting to {connection.user}@{connection.host}...")
            connection.open()
            print_debug(f"Connected to {connection.user}@{connection.host}...")
            if robot is not None:
                connection.original_host = robot.name
            else:
                print_debug(f"Getting hostname of unknown target {connection.host}...")
                hostname: str = connection.run("hostname", hide=hide_output()).stdout.strip()
                print_debug(f"Got hostname of {connection.host}: {hostname}. Using it as name.")
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
    failed_targets: list[tuple[str, list[tuple[str, str]]]] = []

    for candidate in target_candidates:
        target = candidate.label
        target_ips = candidate.ips
        if len(target_ips) > 1:
            print_debug(f"Target '{target}' resolved to multiple candidate IPs: {', '.join(target_ips)}.")

        results_by_ip: dict[str, tuple[Connection | None, str | None]] = {}
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(target_ips)) as executor:
            futures = [executor.submit(_try_connect, target_ip, candidate.robot) for target_ip in target_ips]
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
            open_connections.append(selected_connection)
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


def get_connections_from_targets(targets: list[str], user: str, connection_timeout: int | None = 10) -> ThreadingGroup:
    """
    Parses the given targets, then creates connections to the parsed hosts.
    NOTE: If targets is 'ALL', all known targets will be used and failed connections will be ignored.

    :param targets: List of target strings (either robot numbers, names, aliases or IPs, or 'ALL'). If 'ALL' is included, it will be expanded to all known robots.
    :param user: The username to connect with
    :param connection_timeout: Timeout for establishing the connection
    :return: The connections to the targets
    """
    if _ALL_TARGETS in targets:
        print_info(f"Connecting to all known robots: {', '.join(robot.name for robot in KNOWN_ROBOTS)}")
        target_candidates = [
            TargetCandidate(target=robot.name, ips=list(robot.ips), robot=robot) for robot in KNOWN_ROBOTS
        ]
    else:
        target_candidates = _parse_target_candidates(targets)

    return _get_connections_from_target_candidates(
        target_candidates=target_candidates, user=user, connection_timeout=connection_timeout
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
