import sys
from pathlib import Path

_SCRIPTS_PATH = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SCRIPTS_PATH))

from deploy import pixi  # noqa: E402

_PIXI_REPOSITORY_CHECK_COMMAND = "curl -sSfLI --max-time 2 https://data.bit-bots.de/conda-misc/output/"


def test_pixi_run_command_uses_as_is_when_robot_is_offline():
    connection = _FakeConnection(internet_available=False)

    command = pixi.pixi_run_command(connection, "~/bitbots/bitbots_main", "build bitbots_utils")

    assert command == "cd ~/bitbots/bitbots_main && pixi run --as-is --environment robot build bitbots_utils"


def test_pixi_run_command_omits_as_is_when_robot_is_online():
    connection = _FakeConnection(internet_available=True)

    command = pixi.pixi_run_command(connection, "~/bitbots/bitbots_main", "build bitbots_utils")

    assert command == "cd ~/bitbots/bitbots_main && pixi run --environment robot build bitbots_utils"


def test_robot_has_internet_caches_result():
    connection = _FakeConnection(internet_available=False)

    assert pixi.robot_has_internet(connection) is False
    assert pixi.robot_has_internet(connection) is False

    assert connection.commands == [_PIXI_REPOSITORY_CHECK_COMMAND]


def test_pixi_install_is_skipped_when_robot_is_offline():
    connection = _FakeConnection(internet_available=False)

    result = pixi.pixi_install_if_online(connection, "~/bitbots/bitbots_main")

    assert result.ok
    assert result.command == "# Skipping pixi install because the robot has no internet connection."
    assert connection.commands == [_PIXI_REPOSITORY_CHECK_COMMAND]


def test_pixi_install_runs_when_robot_is_online():
    connection = _FakeConnection(internet_available=True)

    result = pixi.pixi_install_if_online(connection, "~/bitbots/bitbots_main")

    assert result.ok
    assert connection.commands == [
        _PIXI_REPOSITORY_CHECK_COMMAND,
        "cd ~/bitbots/bitbots_main && pixi install --environment robot",
    ]


class _FakeConnection:
    def __init__(self, internet_available: bool):
        self.host = "10.0.0.1"
        self.original_host = "nuc"
        self.internet_available = internet_available
        self.commands = []

    def run(self, command, hide=None, warn=False):
        self.commands.append(command)
        if command == _PIXI_REPOSITORY_CHECK_COMMAND:
            return _FakeResult(self, command, ok=self.internet_available)
        return _FakeResult(self, command, ok=True)


class _FakeResult:
    def __init__(self, connection: _FakeConnection, command: str, ok: bool):
        self.connection = connection
        self.command = command
        self.ok = ok
        self.failed = not ok
        self.stdout = ""
        self.stderr = ""
        self.exited = int(not ok)
