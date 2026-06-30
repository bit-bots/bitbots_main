import socket
from ipaddress import IPv4Address
from types import SimpleNamespace

import pytest

from bitbots_team_communication import network


def test_resolve_manual_target_ip(mocker):
    auto_detection = mocker.patch.object(network, "get_wifi_broadcast_address")

    target_ip, interface = network.resolve_target_ip("10.0.6.255", in_sim=False)

    assert target_ip == IPv4Address("10.0.6.255")
    assert interface is None
    auto_detection.assert_not_called()


def test_resolve_manual_target_ip_ignores_sim(mocker):
    auto_detection = mocker.patch.object(network, "get_wifi_broadcast_address")

    target_ip, interface = network.resolve_target_ip("10.0.6.255", in_sim=True)

    assert target_ip == IPv4Address("10.0.6.255")
    assert interface is None
    auto_detection.assert_not_called()


def test_resolve_auto_target_ip(mocker):
    mocker.patch.object(
        network,
        "get_wifi_broadcast_address",
        return_value=(IPv4Address("10.0.6.255"), "wlp2s0"),
    )

    target_ip, interface = network.resolve_target_ip("auto", in_sim=False)

    assert target_ip == IPv4Address("10.0.6.255")
    assert interface == "wlp2s0"


def test_resolve_auto_target_ip_in_sim_uses_localhost(mocker):
    auto_detection = mocker.patch.object(network, "get_wifi_broadcast_address")

    target_ip, interface = network.resolve_target_ip("auto", in_sim=True)

    assert target_ip == IPv4Address("127.0.0.1")
    assert interface is None
    auto_detection.assert_not_called()


def test_auto_target_ip_errors_without_wifi_interface(tmp_path, monkeypatch):
    monkeypatch.setattr(network, "SYS_CLASS_NET", tmp_path)
    (tmp_path / "eth0").mkdir()

    with pytest.raises(network.WifiInterfaceError, match="no Wi-Fi interface"):
        network.get_wifi_broadcast_address()


def test_wifi_interface_detection_follows_sysfs_symlink(tmp_path):
    device_interface = tmp_path / "devices" / "wlp2s0"
    (device_interface / "wireless").mkdir(parents=True)
    class_interface = tmp_path / "class" / "net" / "wlp2s0"
    class_interface.parent.mkdir(parents=True)
    class_interface.symlink_to(device_interface)

    assert network._is_wifi_interface(class_interface)


def test_auto_target_ip_errors_without_connected_wifi(tmp_path, monkeypatch):
    monkeypatch.setattr(network, "SYS_CLASS_NET", tmp_path)
    _create_wifi_interface(tmp_path, "wlan0", carrier="0")

    with pytest.raises(network.WifiInterfaceError, match="no connected Wi-Fi interface"):
        network.get_wifi_broadcast_address()


def test_auto_target_ip_errors_without_wifi_ipv4_address(tmp_path, monkeypatch, mocker):
    monkeypatch.setattr(network, "SYS_CLASS_NET", tmp_path)
    _create_wifi_interface(tmp_path, "wlan0", carrier="1")
    mocker.patch.object(network, "_get_ipv4_broadcast_address", return_value=None)

    with pytest.raises(network.WifiInterfaceError, match="no connected Wi-Fi interface"):
        network.get_wifi_broadcast_address()


def test_auto_target_ip_uses_connected_wifi_broadcast(tmp_path, monkeypatch, mocker):
    monkeypatch.setattr(network, "SYS_CLASS_NET", tmp_path)
    _create_wifi_interface(tmp_path, "wlp2s0", carrier="1")
    (tmp_path / "eth0").mkdir()
    get_broadcast_address = mocker.patch.object(
        network,
        "_get_ipv4_broadcast_address",
        return_value=IPv4Address("192.168.1.255"),
    )

    assert network.get_wifi_broadcast_address() == (IPv4Address("192.168.1.255"), "wlp2s0")
    get_broadcast_address.assert_called_once_with("wlp2s0")


def test_auto_target_ip_errors_for_multiple_connected_wifi_interfaces(tmp_path, monkeypatch, mocker):
    monkeypatch.setattr(network, "SYS_CLASS_NET", tmp_path)
    _create_wifi_interface(tmp_path, "wlan0", carrier="1")
    _create_wifi_interface(tmp_path, "wlan1", carrier="1")
    mocker.patch.object(
        network,
        "_get_ipv4_broadcast_address",
        side_effect=[IPv4Address("10.0.6.255"), IPv4Address("192.168.1.255")],
    )

    with pytest.raises(network.WifiInterfaceError, match="multiple connected Wi-Fi interfaces"):
        network.get_wifi_broadcast_address()


def test_get_ipv4_broadcast_address_uses_psutil(mocker):
    mocker.patch.object(
        network.psutil,
        "net_if_addrs",
        return_value={
            "wlan0": [
                SimpleNamespace(family=socket.AF_INET6, broadcast=None),
                SimpleNamespace(family=socket.AF_INET, broadcast="10.0.6.255"),
            ]
        },
    )

    assert network._get_ipv4_broadcast_address("wlan0") == IPv4Address("10.0.6.255")


def test_get_ipv4_broadcast_address_ignores_unspecified_broadcast(mocker):
    mocker.patch.object(
        network.psutil,
        "net_if_addrs",
        return_value={"wlan0": [SimpleNamespace(family=socket.AF_INET, broadcast="0.0.0.0")]},
    )

    assert network._get_ipv4_broadcast_address("wlan0") is None


def test_get_ipv4_broadcast_address_returns_none_without_matching_interface(mocker):
    mocker.patch.object(network.psutil, "net_if_addrs", return_value={})

    assert network._get_ipv4_broadcast_address("wlan0") is None


def _create_wifi_interface(root, name: str, carrier: str):
    interface = root / name
    (interface / "wireless").mkdir(parents=True)
    (interface / "carrier").write_text(carrier)
    return interface
