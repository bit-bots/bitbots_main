from typing import Any, Optional

import ipaddress
import os
import subprocess

import yaml

from rich.console import Console
from rich.panel import Panel
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


# Read the known targets
_known_targets_path: str = os.path.join(os.path.dirname(__file__), "known_targets.yaml")
try:
    with open(_known_targets_path, "r") as f:
        KNOWN_TARGETS: dict[str, dict[str, str]] = yaml.safe_load(f)
except FileNotFoundError:
    print_err(f"Could not find known_targets.yaml in {_known_targets_path}")
    exit(1)


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
        identified_target: Optional[str] = None  # The hostname of the identified target

        # Iterate over the known targets
        for hostname, values in KNOWN_TARGETS.items():
            # Is the identifier a known hostname?
            if hostname == identifier:
                identified_target = hostname
                break

            # Is the identifier a known robot name?
            elif values.get("robot_name") == identifier:
                identified_target = hostname
                break

            # Is the identifier a known IP address?
            else:
                try:
                    identifier_ip = ipaddress.ip_address(identifier)
                except ValueError:
                    print_err(f"Could not find a known target for the given identifier: {identifier}")
                    exit(1)

                if "ip" in values:
                    try:
                        known_target_ip = ipaddress.ip_address(values["ip"])
                    except ValueError:
                        print_err(f"Invalid IP address defined for known target: {hostname}")
                        exit(1)

                    if identifier_ip == known_target_ip:
                        identified_target = hostname
                        break

        # If no target was identified, exit
        if identified_target is None:
            print_err(f"Could not find a known target for the given identifier: {identifier}")
            exit(1)

        identified_ip = None
        if "ip" in KNOWN_TARGETS[identified_target]:
            try:
                identified_ip = ipaddress.ip_address(KNOWN_TARGETS[identified_target]["ip"])
            except ValueError:
                print_err(f"Invalid IP address defined for known target: {identified_target}")
                exit(1)

        return (identified_target, identified_ip)


    def __str__(self) -> str:
        """Returns the target's hostname if available or IP-address."""
        return self.hostname if self.hostname is not None else str(self.ip)
