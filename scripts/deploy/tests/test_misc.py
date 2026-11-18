import importlib.util
import re
import subprocess
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
def known_robots(monkeypatch):
    robots = misc._parse_known_robots(
        [
            {"name": "Kalliope", "number": 1, "ips": ["172.20.1.11", "10.0.6.1", "10.1.24.125"]},
            {"name": "Mickey", "number": 2, "aliases": ["mouse"], "ips": ["172.20.1.12"]},
            {"name": "Romeo", "number": 4, "ips": ["172.20.1.14"]},
        ]
    )
    monkeypatch.setattr(misc, "KNOWN_ROBOTS", robots)
    return robots


def test_known_targets_file_is_valid():
    path = Path(misc.__file__).parent / "known_targets.yaml"
    assert misc._load_known_robots(str(path))


@pytest.mark.parametrize("target", ["Kalliope", "kalliope", "KALLIOPE", " kalliope ", "1"])
def test_parse_targets_returns_all_ips_of_robot(known_robots, target):
    assert misc._parse_targets([target]) == ["172.20.1.11", "10.0.6.1", "10.1.24.125"]


@pytest.mark.parametrize("target", ["mouse", "Mouse"])
def test_parse_targets_resolves_alias(known_robots, target):
    assert misc._parse_targets([target]) == ["172.20.1.12"]


def test_parse_targets_keeps_literal_ip_as_single_target(known_robots):
    [candidate] = misc._parse_target_candidates(["10.0.6.1"])
    assert candidate.ips == ["10.0.6.1"]
    assert candidate.label == "Kalliope"


def test_parse_targets_accepts_unknown_ip(known_robots):
    [candidate] = misc._parse_target_candidates(["192.168.1.5"])
    assert candidate.ips == ["192.168.1.5"]
    assert candidate.robot is None
    assert candidate.label == "192.168.1.5"


def test_parse_targets_handles_mixed_targets(known_robots):
    assert misc._parse_targets(["1", "romeo", "2"]) == [
        "172.20.1.11",
        "10.0.6.1",
        "10.1.24.125",
        "172.20.1.14",
        "172.20.1.12",
    ]


def test_parse_targets_skips_aliases_of_already_given_robot(known_robots):
    assert misc._parse_targets(["1", "kalliope", "172.20.1.11"]) == [
        "172.20.1.11",
        "10.0.6.1",
        "10.1.24.125",
    ]


def test_parse_targets_skips_robot_already_given_by_ip(known_robots):
    assert misc._parse_targets(["10.0.6.1", "kalliope", "4"]) == ["10.0.6.1", "172.20.1.14"]


def test_parse_targets_skips_repeated_unknown_ip(known_robots):
    assert misc._parse_targets(["192.168.1.5", "192.168.1.5"]) == ["192.168.1.5"]


def test_parse_targets_exits_for_unknown_target(known_robots):
    with pytest.raises(SystemExit):
        misc._parse_targets(["unknown"])


@pytest.mark.parametrize(
    ("robots", "error"),
    [
        ({"name": "Kalliope"}, "list of robots"),
        ([{"name": "Kalliope", "number": 1}], "'ips'"),
        ([{"name": "Kalliope", "number": 1, "ips": []}], "'ips'"),
        ([{"name": "Kalliope", "number": 1, "ips": ["not-an-ip"]}], "invalid IP"),
        ([{"name": "Kalliope", "number": "1", "ips": ["10.0.6.1"]}], "integer 'number'"),
        ([{"name": "Kalliope", "number": 1, "ip": ["10.0.6.1"]}], "unknown keys: ip"),
        ([{"name": "Kalliope", "number": 1, "aliases": ["2"], "ips": ["10.0.6.1"]}], "ambiguous"),
        (
            [
                {"name": "Kalliope", "number": 1, "ips": ["10.0.6.1"]},
                {"name": "Mickey", "number": 2, "aliases": ["KALLIOPE"], "ips": ["10.0.6.2"]},
            ],
            "already used by robot 'Kalliope'",
        ),
        (
            [
                {"name": "Kalliope", "number": 1, "ips": ["10.0.6.1"]},
                {"name": "Mickey", "number": 1, "ips": ["10.0.6.2"]},
            ],
            "Number 1",
        ),
        (
            [
                {"name": "Kalliope", "number": 1, "ips": ["10.0.6.1"]},
                {"name": "Mickey", "number": 2, "ips": ["10.0.6.1"]},
            ],
            "IP address 10.0.6.1",
        ),
    ],
)
def test_parse_known_robots_rejects_invalid_entries(robots, error):
    with pytest.raises(ValueError, match=re.escape(error)):
        misc._parse_known_robots(robots)


def test_get_connections_names_connections_after_robot_or_hostname(monkeypatch, known_robots):
    monkeypatch.setattr(misc, "Connection", _fake_connection_factory(failing_hosts=set()))
    monkeypatch.setattr(misc.ThreadingGroup, "from_connections", lambda connections: list(connections))

    connections = misc._get_connections_from_target_candidates(
        misc._parse_target_candidates(["romeo", "192.168.1.5"]), user="bitbots"
    )

    assert [connection.original_host for connection in connections] == ["Romeo", "192.168.1.5-hostname"]
    assert connections[0].commands == []
    assert connections[1].commands == ["hostname"]


def test_get_connections_for_all_uses_one_connection_per_robot(monkeypatch, known_robots):
    monkeypatch.setattr(misc, "Connection", _fake_connection_factory(failing_hosts={"172.20.1.11"}))
    monkeypatch.setattr(misc.ThreadingGroup, "from_connections", lambda connections: list(connections))

    connections = misc.get_connections_from_targets(["ALL"], user="bitbots")

    assert [(connection.original_host, connection.host) for connection in connections] == [
        ("Kalliope", "10.0.6.1"),
        ("Mickey", "172.20.1.12"),
        ("Romeo", "172.20.1.14"),
    ]


def test_get_connections_uses_reachable_candidate_ip(monkeypatch):
    warnings = []
    infos = []
    monkeypatch.setattr(misc, "Connection", _fake_connection_factory(failing_hosts={"172.20.1.12"}))
    monkeypatch.setattr(misc.ThreadingGroup, "from_connections", lambda connections: list(connections))
    monkeypatch.setattr(misc, "print_warning", warnings.append)
    monkeypatch.setattr(misc, "print_info", infos.append)

    connections = misc._get_connections_from_target_candidates(
        [_candidate("mickey", ["172.20.1.12", "10.0.6.2"])], user="nvidia"
    )

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
        [_candidate("mickey", ["172.20.1.12", "10.0.6.2", "10.1.24.153"])], user="nvidia"
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
        misc._get_connections_from_target_candidates([_candidate("mickey", ["172.20.1.12", "10.0.6.2"])], user="nvidia")

    assert len(warnings) == 1
    assert errors == ["Could not establish any connection to the given targets. Exiting..."]


def test_get_connections_continues_when_another_target_has_reachable_ip(monkeypatch):
    warnings = []
    monkeypatch.setattr(misc, "Connection", _fake_connection_factory(failing_hosts={"172.20.1.12", "10.0.6.2"}))
    monkeypatch.setattr(misc.ThreadingGroup, "from_connections", lambda connections: list(connections))
    monkeypatch.setattr(misc, "print_warning", warnings.append)

    connections = misc._get_connections_from_target_candidates(
        [
            _candidate("mickey", ["172.20.1.12", "10.0.6.2"]),
            _candidate("kalliope", ["172.20.1.11"]),
        ],
        user="nvidia",
    )

    assert [connection.host for connection in connections] == ["172.20.1.11"]
    assert len(warnings) == 1


def _candidate(target: str, ips: list[str]):
    return misc.TargetCandidate(target=target, ips=ips, robot=None)


def _fake_connection_factory(failing_hosts: set[str], failing_host_delay: float = 0.0):
    class FakeConnection:
        def __init__(self, host, user, connect_timeout):
            self.host = host
            self.user = user
            self.connect_timeout = connect_timeout
            self.closed = False
            self.original_host = host
            self.commands = []

        def open(self):
            if self.host in failing_hosts:
                if failing_host_delay:
                    time.sleep(failing_host_delay)
                raise TimeoutError("timed out")

        def run(self, command, hide):
            assert command == "hostname"
            self.commands.append(command)
            return _FakeRunResult(stdout=f"{self.host}-hostname\n")

        def close(self):
            self.closed = True

    return FakeConnection


class _FakeRunResult:
    def __init__(self, stdout: str):
        self.stdout = stdout


def test_check_ssh_configuration_accepts_valid_configuration(monkeypatch):
    monkeypatch.setattr(
        misc.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(args[0], returncode=0, stdout="configuration", stderr=""),
    )

    misc.check_ssh_configuration("bitbots", "robot")


def test_check_ssh_configuration_explains_unsafe_permissions(monkeypatch):
    config_path = "/home/user/.ssh/config"
    errors = []
    monkeypatch.setattr(
        misc.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0], returncode=255, stdout="", stderr=f"Bad owner or permissions on {config_path}\n"
        ),
    )
    monkeypatch.setattr(misc, "print_error", errors.append)

    with pytest.raises(subprocess.CalledProcessError):
        misc.check_ssh_configuration("bitbots", "robot")

    assert errors == [
        f"OpenSSH rejected the configuration file '{config_path}' because its owner or permissions are unsafe. "
        "Ensure that you own the file and remove write permissions for group and others, for example with "
        f"'chmod go-w {config_path}'."
    ]


def test_check_ssh_configuration_reports_other_errors(monkeypatch):
    errors = []
    monkeypatch.setattr(
        misc.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0], returncode=255, stdout="", stderr="Could not resolve hostname robot"
        ),
    )
    monkeypatch.setattr(misc, "print_error", errors.append)

    with pytest.raises(subprocess.CalledProcessError):
        misc.check_ssh_configuration("bitbots", "robot")

    assert errors == ["OpenSSH could not read the SSH configuration:\nCould not resolve hostname robot"]
