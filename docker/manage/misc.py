import os
from pathlib import Path
from typing import Any

import yaml
from rich import box
from rich.console import Console
from rich.panel import Panel

CONSOLE = Console()

# Constants
IMAGE_NAME_BASE = "bitbots-base"
IMAGE_NAME_PROJECT = "bitbots-project"
DEFAULT_USER = "bitbots"
SSH_PORT_PROJECT = 2222
NETWORK_NAME = "bitbots-net"
DEFAULT_SUBNET = "10.66.0.0/16"

# Root paths
DOCKER_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = DOCKER_DIR.parent
KNOWN_TARGETS_PATH = REPO_ROOT / "scripts" / "deploy" / "known_targets.yaml"


class LOGLEVEL:
    ERR_SUCCESS = 0
    WARN = 1
    INFO = 2
    DEBUG = 3
    CURRENT = 2


def print_error(msg: Any) -> None:
    """Prints an error message in a red box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.ERR_SUCCESS:
        CONSOLE.print(Panel(str(msg), title="Error", style="bold red", box=box.HEAVY))


def print_warning(msg: Any) -> None:
    """Prints a warning message in a yellow box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.WARN:
        CONSOLE.print(Panel(str(msg), title="Warning", style="yellow", box=box.SQUARE))


def print_success(msg: Any) -> None:
    """Prints a success message in a green box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.ERR_SUCCESS:
        CONSOLE.print(Panel(str(msg), title="Success", style="green", box=box.SQUARE))


def print_info(msg: Any) -> None:
    """Prints an info message to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.INFO:
        CONSOLE.log(msg, style="")


def print_debug(msg: Any) -> None:
    """Prints a debug message to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.DEBUG:
        CONSOLE.log(msg, style="dim")


def print_bit_bot() -> None:
    """Prints the Bit-Bots logo to the console if available."""
    logo_path = REPO_ROOT / "scripts" / "deploy" / "bitbot.ans"
    if logo_path.is_file():
        CONSOLE.print(logo_path.read_text())


def get_known_targets(path: Path | str = KNOWN_TARGETS_PATH) -> dict[str, dict[str, Any]]:
    """Loads and returns the known targets dictionary from YAML."""
    path = Path(path)
    if not path.is_file():
        print_debug(f"known_targets.yaml not found at {path}")
        return {}
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
            return data if isinstance(data, dict) else {}
    except Exception as e:
        print_debug(f"Error loading {path}: {e}")
        return {}


def resolve_robot_ip(
    identifier: str,
    subnet: str = DEFAULT_SUBNET,
    known_targets_path: Path | str = KNOWN_TARGETS_PATH,
) -> str | None:
    """Resolves a robot IP address from an identifier (hostname, robot name, or IP)."""
    if not identifier:
        return None

    import ipaddress

    ident = identifier.strip().lower()

    # If identifier is already a valid IP address
    try:
        ipaddress.ip_address(ident)
        return ident
    except ValueError:
        pass

    targets = get_known_targets(known_targets_path)
    try:
        net = ipaddress.ip_network(subnet)
    except ValueError:
        net = None

    matches = []
    for ip_str, info in targets.items():
        hostname = str(info.get("hostname", "")).lower()
        robot_name = str(info.get("robot_name", "")).lower()
        if hostname == ident or robot_name == ident or ip_str.lower() == ident:
            matches.append(ip_str)

    # Prefer IP within the specified subnet
    if net:
        for m in matches:
            try:
                if ipaddress.ip_address(m) in net:
                    return m
            except ValueError:
                pass

    if matches:
        return matches[0]

    return None


def resolve_robot_domain_id(
    identifier: str,
    subnet: str = DEFAULT_SUBNET,
    known_targets_path: Path | str = KNOWN_TARGETS_PATH,
) -> str | None:
    """Resolves a robot's ROS domain ID from an identifier."""
    if not identifier:
        return None

    import ipaddress

    ident = identifier.strip().lower()
    targets = get_known_targets(known_targets_path)

    try:
        net = ipaddress.ip_network(subnet)
    except ValueError:
        net = None

    matches = []
    for ip_str, info in targets.items():
        hostname = str(info.get("hostname", "")).lower()
        robot_name = str(info.get("robot_name", "")).lower()
        if hostname == ident or robot_name == ident or ip_str.lower() == ident:
            matches.append((ip_str, info))

    found_info = None
    if net:
        for m_ip, m_info in matches:
            try:
                if ipaddress.ip_address(m_ip) in net:
                    found_info = m_info
                    break
            except ValueError:
                pass

    if not found_info and matches:
        found_info = matches[0][1]

    if found_info:
        domain_id = found_info.get("domain_id")
        if domain_id is None:
            domain_id = found_info.get("ros_domain_id")
        if domain_id is not None:
            return str(domain_id)

    return None


def find_ssh_key(custom_path: str | Path | None = None) -> Path | None:
    """Finds SSH public key path from env, parameter, or standard locations."""
    if custom_path:
        p = Path(custom_path).expanduser()
        if p.is_file():
            return p

    env_path = os.environ.get("SSH_PUB_KEY_PATH")
    if env_path:
        p = Path(env_path).expanduser()
        if p.is_file():
            return p

    home = Path.home()
    for candidate in [home / ".ssh" / "id_ed25519.pub", home / ".ssh" / "id_rsa.pub"]:
        if candidate.is_file():
            return candidate

    return None
