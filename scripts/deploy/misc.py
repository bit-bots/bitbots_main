from typing import Any, Optional

import ipaddress
import os
import sys

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
    """Prints the Bit-Bots logo to the console."""
    CONSOLE.print("""
                `/shNMoyymmmmmmmmmmys+NmNs/`
              `+mmdddmmmddddddddddddmmmdddmm/
              ymdddddddmmmmmmmmmmmmmmdddddddms
            .dmdddddddddddmddddddmmddddddddddmy`
           .mmdddddddddddmyoooooosddddddddddddms`
       .ssshmdddddddddddddmhsooshddddddddddddddmhsss.
    -+shysshddddmddddmdddddddddddddddmdhhhhdmdddhssyhs+-
   :ddyyyyydddy+/////+shddddddddddddy+//////+sdddyyyyydd:
  `mmh/+hdddh///////////sdddddddddd+///++++////hdddh+/hmm`
  omdddddmmy///sdNNNmy///omddddddd+//odNMMNmo///hdmdddddmo
 `dddddddmm+//yMMMMMMMh///hdddmddh//oNMMMMMMNo//smmddddddd`
 .Nddddddmm+//hMMMMMMMd///hdhNNddh//+NMMMMMMNo//ymmddddddN.
 .mNNNNNNNdh///yNMMMNd+//sdddddddds//+hmNNmho//oddmmNNNNNm.
-ydddmmmmNmdh+///+++////yddddddddddy//////////sddmNmmmmdddy-
:sddddddymNdddyo+////oyddddddddddddddys++++oyddddddydddddds:
/hddddddh::Nddddmdddmdddddddddddddddddddmmdddddmm.:hdddddhh/
:dhhhhyhh. :dmddddddddddddddddddddddddddddddddmy. .hhyhhhhd:
/mhhhhyo/   `omdddddddddddddddddddddddddddddmmo`   /oyhhhhm/
/hhhhd`       `ommddddddddddddddddddddddddmmo`       `dhhhh/
/hhhhd+         `+ymdddddddddddddddddddddy:`         +dhhhh/
mhhhhds          +:mhhdhmmmmmddmmmmNmhhhs/:          sdhhhhm
mhhhhm.         .mNhsshmN.-//:///-hNdyoydMm          .mhhhhm
mhhhhs          oNNy//ymN`        yNho:omNd           shhhhm
dhhss`          +NMNsyNNN`        yNNm+dNNd           `sshhd
dhdh/.          +NNdyyNNm`        yNNdyhNNd           ./hdhd
mhssy+          +dddddddm`        ydddddddd           +ysshm
hy-             +dddddddm`        ydddddddd              -yh
               `hmmmmmmmm+       -dmmmmmmmm:
               `Nddddddddy       :mdddddddm+
               `Nddddddddy       :mdddddddm+
               `Ndddddddmy       :mdddddddm+
               `mddddddddy       :mdddddddd+
               `mddddddddy       :mdddddddd+
               `mmNmohNNdy       :mmNmohmNm+
               `NNmdhdmNmh       /NmmdhddNms
              `/mms+-/ymm:        ymho-:odmh/.
              yo+hsoooshs         -hsooosys+y:
              h+++++++oo+         `ho+++++++s:
              dysssssssy+         `dsssssssyh:
""", style="bold white")


class LOGLEVEL:
    ERR_SUCCESS = 0
    WARN = 1
    INFO = 2
    DEBUG = 3
    CURRENT = 2

    def should_run_quietly(self) -> bool:
        """
        Returns whether the task should run quietly or not.

        :return: True if the current loglevel is below INFO, False otherwise.
        """
        return self.CURRENT <= self.INFO


# Read the known targets
_known_targets_path: str = os.path.join(os.path.dirname(__file__), "known_targets.yaml")
try:
    with open(_known_targets_path, "r") as f:
        KNOWN_TARGETS: dict[str, dict[str, str]] = yaml.safe_load(f)
except FileNotFoundError:
    print_err(f"Could not find known_targets.yaml in {_known_targets_path}")
    sys.exit(1)


class Target:
    hostname: str
    ip: Optional[ipaddress.IPv4Address | ipaddress.IPv6Address]
    workspace: Optional[str]

    def __init__(self, identifier: str) -> None:
        """
        Target represents a robot to deploy to.
        It can be initialized with a hostname, IP address or a robot name.
        """
        self.hostname, self.ip, self.workspace = self._identify_target(identifier)

    def _identify_target(self, identifier: str) -> tuple[str, Optional[ipaddress.IPv4Address | ipaddress.IPv6Address], Optional[str]]:
        """
        Identifies a target from an identifier.
        The identifier can be a hostname, IP address or a robot name.

        :param identifier: The identifier to identify the target from.
        :return: A tuple containing the hostname, IP address and workspace of the target.
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
                    sys.exit(1)

                if "ip" in values:
                    try:
                        known_target_ip = ipaddress.ip_address(values["ip"])
                    except ValueError:
                        print_err(f"Invalid IP address defined for known target: {hostname}")
                        sys.exit(1)

                    if identifier_ip == known_target_ip:
                        identified_target = hostname
                        break

        # If no target was identified, exit
        if identified_target is None:
            print_err(f"Could not find a known target for the given identifier: {identifier}")
            sys.exit(1)

        identified_ip = None
        if "ip" in KNOWN_TARGETS[identified_target]:
            try:
                identified_ip = ipaddress.ip_address(KNOWN_TARGETS[identified_target]["ip"])
            except ValueError:
                print_err(f"Invalid IP address defined for known target: {identified_target}")
                sys.exit(1)

        identified_workspace = KNOWN_TARGETS[identified_target].get("workspace")

        return (identified_target, identified_ip, identified_workspace)


    def __str__(self) -> str:
        """Returns the target's hostname if available or IP-address."""
        return self.hostname if self.hostname is not None else str(self.ip)
