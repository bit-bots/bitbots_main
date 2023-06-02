from typing import Dict, Optional, Any

import ipaddress
from collections import defaultdict
from rich.console import Console
from rich.panel import Panel
from rich import box


CONSOLE = Console()


class LOGLEVEL:
    ERR_SUCCESS = 0
    WARN = 1
    INFO = 2
    DEBUG = 3
    CURRENT = 2


class Target:
    # Map robot names to hostnames
    _robotnames: Dict[str, str] = {
        "amy": "nuc1",
        "rory": "nuc2",
        "jack": "nuc3",
        "donna": "nuc4",
        "melody": "nuc5",
        "rose": "nuc6",
    }

    # Map hostnames to workspaces
    _workspaces = defaultdict(lambda: "~/colcon_ws")
    _workspaces["nuc1"] = "~/colcon_ws"
    _workspaces["nuc2"] = "~/colcon_ws"
    _workspaces["nuc3"] = "~/colcon_ws"
    _workspaces["nuc4"] = "~/colcon_ws"
    _workspaces["nuc5"] = "~/colcon_ws"
    _workspaces["nuc6"] = "~/colcon_ws"

    _IP_prefix = "172.20.1."
    # Map hostnames to IPs
    _IPs = {
        "nuc1": ipaddress.ip_address(_IP_prefix + "11"),
        "nuc2": ipaddress.ip_address(_IP_prefix + "12"),
        "nuc3": ipaddress.ip_address(_IP_prefix + "13"),
        "nuc4": ipaddress.ip_address(_IP_prefix + "14"),
        "nuc5": ipaddress.ip_address(_IP_prefix + "15"),
        "nuc6": ipaddress.ip_address(_IP_prefix + "16"),
    }

    def __init__(self, identifier: str) -> None:
        """
        Target represents a robot to deploy to.
        It can be initialized with a hostname, IP or robot name.
        """
        self.hostname: Optional[str] = None
        self.ip: Optional[ipaddress.IPv4Address | ipaddress.IPv6Address] = None

        # Is identifier an IP?
        try:
            self.ip = ipaddress.ip_address(identifier)
            # Infer hostname from IP
            for name, known_ip in self._IPs.items():
                if known_ip == self.ip:
                    self.hostname = name
        except ValueError:
            self.ip = None

        # Is identifier a hostname?
        if identifier in self._IPs.keys():
            self.hostname = identifier

        # Is identifier a robot name?
        if identifier in self._robotnames.keys():
            self.hostname = self._robotnames[identifier]

        if self.hostname is not None and self.hostname in self._IPs.keys():
            self.ip = self._IPs[self.hostname]
        else:
            raise ValueError("Could not determine hostname or IP from input: '{identifier}'")
        
        self.workspace = self._workspaces[self.hostname]

    def __str__(self) -> str:
        """Returns the target's hostname if available or IP-address."""
        return self.hostname if self.hostname is not None else str(self.ip)


def should_run_quietly() -> bool:
    """
    Returns whether the task should run quietly or not.
    
    :return: True if the current loglevel is below INFO, False otherwise.
    """
    return LOGLEVEL.CURRENT <= LOGLEVEL.INFO


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
    print("""\033[1m
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
\033[0m""")

