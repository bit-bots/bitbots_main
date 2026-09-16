import importlib.util
import sys
from pathlib import Path

import pytest

_MISC_PATH = Path(__file__).resolve().parents[1] / "misc.py"
_SPEC = importlib.util.spec_from_file_location("docker_manage_misc", _MISC_PATH)
assert _SPEC is not None
assert _SPEC.loader is not None
misc = importlib.util.module_from_spec(_SPEC)
sys.modules["docker_manage_misc"] = misc
_SPEC.loader.exec_module(misc)


@pytest.fixture
def sample_targets(monkeypatch):
    targets = {
        "10.66.6.1": {"hostname": "minnie", "robot_name": "Minnie", "domain_id": 12},
        "10.66.6.2": {"hostname": "mickey", "robot_name": "Mickey", "ros_domain_id": 11},
        "172.20.1.12": {"hostname": "mickey", "robot_name": "Mickey", "ros_domain_id": 11},
        "10.66.6.10": {"hostname": "simulator", "robot_name": "Simulator", "domain_id": 0},
    }
    monkeypatch.setattr(misc, "get_known_targets", lambda path=None: targets)
    return targets


def test_resolve_robot_ip_by_hostname(sample_targets):
    # Prefers IP in 10.66.0.0/16
    assert misc.resolve_robot_ip("mickey") == "10.66.6.2"
    assert misc.resolve_robot_ip("minnie") == "10.66.6.1"


def test_resolve_robot_ip_by_robot_name(sample_targets):
    assert misc.resolve_robot_ip("Mickey") == "10.66.6.2"
    assert misc.resolve_robot_ip("Minnie") == "10.66.6.1"


def test_resolve_robot_ip_by_direct_ip():
    assert misc.resolve_robot_ip("10.66.6.20") == "10.66.6.20"
    assert misc.resolve_robot_ip("192.168.1.50") == "192.168.1.50"


def test_resolve_robot_ip_unknown(sample_targets):
    assert misc.resolve_robot_ip("unknown_robot") is None
    assert misc.resolve_robot_ip("") is None


def test_resolve_robot_domain_id(sample_targets):
    assert misc.resolve_robot_domain_id("mickey") == "11"
    assert misc.resolve_robot_domain_id("minnie") == "12"
    assert misc.resolve_robot_domain_id("simulator") == "0"
    assert misc.resolve_robot_domain_id("unknown") is None


def test_find_ssh_key_from_env(monkeypatch, tmp_path):
    key_file = tmp_path / "custom_key.pub"
    key_file.write_text("ssh-ed25519 AAAAC3NzaC1lZDI1NTE5 custom")
    monkeypatch.setenv("SSH_PUB_KEY_PATH", str(key_file))

    found = misc.find_ssh_key()
    assert found == key_file


def test_find_ssh_key_custom_param(tmp_path):
    key_file = tmp_path / "param_key.pub"
    key_file.write_text("ssh-rsa AAAAB3NzaC1yc2E param")

    found = misc.find_ssh_key(custom_path=str(key_file))
    assert found == key_file


def test_find_ssh_key_not_found(monkeypatch, tmp_path):
    monkeypatch.delenv("SSH_PUB_KEY_PATH", raising=False)
    monkeypatch.setattr(Path, "home", lambda: tmp_path)

    found = misc.find_ssh_key()
    assert found is None


def test_get_ssh_temp_dir(monkeypatch, tmp_path):
    monkeypatch.setattr("tempfile.gettempdir", lambda: str(tmp_path))
    monkeypatch.setattr("getpass.getuser", lambda: "testuser")

    temp_dir = misc.get_ssh_temp_dir()
    assert temp_dir == tmp_path / "bitbots_ssh_testuser"
    assert temp_dir.is_dir()


def test_get_or_create_ssh_key_pair(monkeypatch, tmp_path):
    # Test generation when keys do not exist
    call_args = []

    def mock_run(cmd, check=True, capture_output=True):
        call_args.append(cmd)
        # Create dummy key files
        Path(cmd[6]).write_text("dummy-private-key")
        Path(f"{cmd[6]}.pub").write_text("dummy-public-key")

    monkeypatch.setattr("subprocess.run", mock_run)
    priv_key, pub_key = misc.get_or_create_ssh_key_pair(temp_dir=tmp_path)

    assert priv_key == tmp_path / "id_ed25519"
    assert pub_key == tmp_path / "id_ed25519.pub"
    assert priv_key.read_text() == "dummy-private-key"
    assert pub_key.read_text() == "dummy-public-key"
    assert len(call_args) == 1
    assert call_args[0][0] == "ssh-keygen"

    # Test re-use when keys already exist
    call_args.clear()
    priv_key2, pub_key2 = misc.get_or_create_ssh_key_pair(temp_dir=tmp_path)
    assert priv_key2 == priv_key
    assert pub_key2 == pub_key
    assert len(call_args) == 0
