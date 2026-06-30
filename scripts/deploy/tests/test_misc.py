import importlib.util
import sys
import time
from pathlib import Path

import pytest

_MISC_PATH = Path(__file__).resolve().parents[1] / "misc.py"
_SPEC = importlib.util.spec_from_file_location("deploy_misc", _MISC_PATH)
assert _SPEC is not None
assert _SPEC.loader is not None
misc = importlib.util.module_from_spec(_SPEC)
sys.modules["deploy_misc"] = misc
_SPEC.loader.exec_module(misc)


@pytest.fixture
def known_targets(monkeypatch):
    targets = {
        "172.20.1.11": {"hostname": "kalliope", "robot_name": "Kalliope"},
        "10.0.6.1": {"hostname": "kalliope", "robot_name": "Kalliope"},
        "10.1.24.125": {"hostname": "kalliope", "robot_name": "Kalliope"},
        "172.20.1.12": {"hostname": "mickey", "robot_name": "mickey"},
    }
    monkeypatch.setattr(misc, "KNOWN_TARGETS", targets)
    return targets


def test_parse_targets_returns_all_ips_for_duplicate_hostname(known_targets):
    assert misc._parse_targets(["kalliope"]) == ["172.20.1.11", "10.0.6.1", "10.1.24.125"]


def test_parse_targets_returns_all_ips_for_duplicate_robot_name(known_targets):
    assert misc._parse_targets(["Kalliope"]) == ["172.20.1.11", "10.0.6.1", "10.1.24.125"]


def test_parse_targets_keeps_literal_ip_as_single_target(known_targets):
    assert misc._parse_targets(["10.0.6.1"]) == ["10.0.6.1"]


def test_parse_targets_does_not_duplicate_ip_when_hostname_and_robot_name_match(known_targets):
    assert misc._parse_targets(["mickey"]) == ["172.20.1.12"]


def test_parse_targets_exits_for_unknown_target(known_targets):
    with pytest.raises(SystemExit):
        misc._parse_targets(["unknown"])


def test_get_connections_uses_reachable_candidate_ip(monkeypatch):
    warnings = []
    infos = []
    monkeypatch.setattr(misc, "Connection", _fake_connection_factory(failing_hosts={"172.20.1.12"}))
    monkeypatch.setattr(misc.ThreadingGroup, "from_connections", lambda connections: list(connections))
    monkeypatch.setattr(misc, "print_warning", warnings.append)
    monkeypatch.setattr(misc, "print_info", infos.append)

    connections = misc._get_connections_from_target_candidates([("mickey", ["172.20.1.12", "10.0.6.2"])], user="nvidia")

    assert [connection.host for connection in connections] == ["10.0.6.2"]
    assert warnings == []
    assert infos == ["Connected to target 'mickey' via 10.0.6.2. Ignoring other candidate IPs: 172.20.1.12."]


def test_get_connections_tests_candidate_ips_in_parallel(monkeypatch):
    monkeypatch.setattr(
        misc,
        "Connection",
        _fake_connection_factory(
            failing_hosts={"172.20.1.12", "10.0.6.2"},
            failing_host_delay=0.2,
        ),
    )
    monkeypatch.setattr(misc.ThreadingGroup, "from_connections", lambda connections: list(connections))

    start_time = time.monotonic()
    connections = misc._get_connections_from_target_candidates(
        [("mickey", ["172.20.1.12", "10.0.6.2", "10.1.24.153"])], user="nvidia"
    )

    assert [connection.host for connection in connections] == ["10.1.24.153"]
    assert time.monotonic() - start_time < 0.35


def test_get_connections_warns_when_all_candidate_ips_fail(monkeypatch):
    warnings = []
    errors = []
    monkeypatch.setattr(misc, "Connection", _fake_connection_factory(failing_hosts={"172.20.1.12", "10.0.6.2"}))
    monkeypatch.setattr(misc.ThreadingGroup, "from_connections", lambda connections: list(connections))
    monkeypatch.setattr(misc, "print_warning", warnings.append)
    monkeypatch.setattr(misc, "print_error", errors.append)

    with pytest.raises(SystemExit):
        misc._get_connections_from_target_candidates([("mickey", ["172.20.1.12", "10.0.6.2"])], user="nvidia")

    assert len(warnings) == 1
    assert errors == ["Could not establish any connection to the given targets. Exiting..."]


def test_get_connections_continues_when_another_target_has_reachable_ip(monkeypatch):
    warnings = []
    monkeypatch.setattr(misc, "Connection", _fake_connection_factory(failing_hosts={"172.20.1.12", "10.0.6.2"}))
    monkeypatch.setattr(misc.ThreadingGroup, "from_connections", lambda connections: list(connections))
    monkeypatch.setattr(misc, "print_warning", warnings.append)

    connections = misc._get_connections_from_target_candidates(
        [
            ("mickey", ["172.20.1.12", "10.0.6.2"]),
            ("kalliope", ["172.20.1.11"]),
        ],
        user="nvidia",
    )

    assert [connection.host for connection in connections] == ["172.20.1.11"]
    assert len(warnings) == 1


def _fake_connection_factory(failing_hosts: set[str], failing_host_delay: float = 0.0):
    class FakeConnection:
        def __init__(self, host, user, connect_timeout):
            self.host = host
            self.user = user
            self.connect_timeout = connect_timeout
            self.closed = False

        def open(self):
            if self.host in failing_hosts:
                if failing_host_delay:
                    time.sleep(failing_host_delay)
                raise TimeoutError("timed out")

        def run(self, command, hide):
            assert command == "hostname"
            return _FakeRunResult(stdout=f"{self.host}-hostname\n")

        def close(self):
            self.closed = True

    return FakeConnection


class _FakeRunResult:
    def __init__(self, stdout: str):
        self.stdout = stdout
