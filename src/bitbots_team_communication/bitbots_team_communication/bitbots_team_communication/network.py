import socket
from ipaddress import IPv4Address
from pathlib import Path

import psutil

SYS_CLASS_NET = Path("/sys/class/net")


class WifiInterfaceError(RuntimeError):
    pass


def resolve_target_ip(configured_target_ip: str) -> tuple[IPv4Address, str | None]:
    if configured_target_ip.strip().lower() != "auto":
        return IPv4Address(configured_target_ip), None

    return get_wifi_broadcast_address()


def get_wifi_broadcast_address() -> tuple[IPv4Address, str]:
    wifi_interfaces = [path for path in SYS_CLASS_NET.iterdir() if _is_wifi_interface(path)]
    if not wifi_interfaces:
        raise WifiInterfaceError("target_ip is 'auto', but no Wi-Fi interface was found")

    connected_interfaces: list[tuple[IPv4Address, str]] = []
    for interface_path in wifi_interfaces:
        if not _has_carrier(interface_path):
            continue

        broadcast_address = _get_ipv4_broadcast_address(interface_path.name)
        if broadcast_address is not None:
            connected_interfaces.append((broadcast_address, interface_path.name))

    if not connected_interfaces:
        names = ", ".join(sorted(path.name for path in wifi_interfaces))
        raise WifiInterfaceError(
            "target_ip is 'auto', but no connected Wi-Fi interface with an IPv4 broadcast address was found "
            f"(detected: {names})"
        )

    if len(connected_interfaces) > 1:
        interfaces = ", ".join(
            f"{interface} ({broadcast_address})" for broadcast_address, interface in sorted(connected_interfaces)
        )
        raise WifiInterfaceError(
            "target_ip is 'auto', but multiple connected Wi-Fi interfaces were found: "
            f"{interfaces}. Set target_ip manually to disambiguate."
        )

    return connected_interfaces[0]


def _is_wifi_interface(interface_path: Path) -> bool:
    return (interface_path / "wireless").is_dir() or (interface_path / "phy80211").exists()


def _has_carrier(interface_path: Path) -> bool:
    """Check if the interface has a carrier (i.e., is connected). Returns False if the carrier file cannot be read."""
    try:
        return (interface_path / "carrier").read_text().strip() == "1"
    except OSError:
        return False


def _get_ipv4_broadcast_address(interface_name: str) -> IPv4Address | None:
    for address in psutil.net_if_addrs().get(interface_name, []):
        if address.family != socket.AF_INET or address.broadcast is None:
            continue

        broadcast_address = IPv4Address(address.broadcast)
        return None if broadcast_address.is_unspecified else broadcast_address

    return None
