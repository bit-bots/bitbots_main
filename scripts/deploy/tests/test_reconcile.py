import asyncio
import subprocess
from pathlib import Path

import pytest
from deploy.artifacts import EnvironmentArtifact
from deploy.models import (
    BallProfile,
    CommandFailedError,
    CommandResult,
    DesiredState,
    MatchProfile,
    RobotTarget,
    Step,
    StepState,
)
from deploy.reconcile import RobotReconciler, robot_id, shell_path, source_fingerprint


class FakeTransport:
    def __init__(self, fail_wifi: bool = False) -> None:
        self.commands: list[tuple[str, str | None]] = []
        self.fail_wifi = fail_wifi
        self.closed = False
        self.configuration_started: asyncio.Event | None = None
        self.release_configuration: asyncio.Event | None = None

    async def connect(self) -> None:
        return None

    async def check(self) -> bool:
        return True

    async def run(self, command: str, *, stdin=None, output=None, check=True):
        self.commands.append((command, stdin))
        if "game_settings.yaml.tmp" in command and self.configuration_started is not None:
            self.configuration_started.set()
            assert self.release_configuration is not None
            await self.release_configuration.wait()
        if self.fail_wifi and "nmcli" in command:
            result = CommandResult(("ssh", command), 1, stderr="profile not found")
            raise CommandFailedError(result)
        return CommandResult(("ssh", command), 0)

    async def rsync(self, source, destination, exclude_file, output=None):
        self.commands.append(("rsync", None))
        return CommandResult(("rsync",), 0)

    async def attach_tmux(self, session="teamplayer"):
        return 0

    def cancel_current(self) -> None:
        return None

    async def close(self) -> None:
        self.closed = True


def desired(generation: int, wifi_profile: str = "Lab") -> DesiredState:
    match = MatchProfile(
        "lab",
        "labor",
        "kid",
        wifi_profile,
        {
            "team_id": 6,
            "team_color": 0,
            "role": "offense",
            "position_number": 0,
            "monitoring_host_ip": "172.20.0.10",
        },
    )
    return DesiredState(
        generation=generation,
        match=match,
        ball=BallProfile("kid", 0.14),
        sync_source=False,
        build=False,
        launch=False,
    )


def test_reconciles_independently_without_environment(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("deploy.reconcile.source_fingerprint", lambda _: "source")
    transport = FakeTransport()
    reconciler = RobotReconciler(
        RobotTarget("nuc2", "127.0.0.1"),
        tmp_path,
        None,
        None,
        retry_interval=0.01,
        transport=transport,
    )

    async def run() -> None:
        reconciler.start()
        reconciler.apply(desired(1))
        for _ in range(100):
            if reconciler.status.applied_generation == 1:
                break
            await asyncio.sleep(0.01)
        assert reconciler.status.applied_generation == 1
        assert reconciler.status.steps[Step.CONFIGURATION].state == StepState.SUCCEEDED
        assert any(stdin and "bot_id: 2" in stdin for _, stdin in transport.commands)
        await reconciler.close()

    asyncio.run(asyncio.wait_for(run(), timeout=3))


def test_deterministic_failure_pauses_until_new_generation(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("deploy.reconcile.source_fingerprint", lambda _: "source")
    transport = FakeTransport(fail_wifi=True)
    reconciler = RobotReconciler(
        RobotTarget("nuc1", "127.0.0.1"),
        tmp_path,
        None,
        None,
        retry_interval=0.01,
        transport=transport,
    )

    async def run() -> None:
        reconciler.start()
        reconciler.apply(desired(1))
        for _ in range(100):
            if reconciler.status.steps[Step.WIFI].state == StepState.FAILED:
                break
            await asyncio.sleep(0.01)
        attempts = sum("nmcli" in command for command, _ in transport.commands)
        await asyncio.sleep(0.05)
        assert sum("nmcli" in command for command, _ in transport.commands) == attempts

        transport.fail_wifi = False
        reconciler.apply(desired(2))
        for _ in range(100):
            if reconciler.status.applied_generation == 2:
                break
            await asyncio.sleep(0.01)
        assert reconciler.status.applied_generation == 2
        await reconciler.close()

    asyncio.run(asyncio.wait_for(run(), timeout=3))


def test_paths_and_robot_ids() -> None:
    assert shell_path("~/bitbots/main") == '"$HOME"/bitbots/main'
    assert robot_id(RobotTarget("nuc6", "host")) == 6


def test_new_generation_supersedes_pending_steps(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("deploy.reconcile.source_fingerprint", lambda _: "source")
    transport = FakeTransport()
    reconciler = RobotReconciler(
        RobotTarget("nuc1", "127.0.0.1"),
        tmp_path,
        None,
        None,
        retry_interval=0.01,
        transport=transport,
    )

    async def run() -> None:
        transport.configuration_started = asyncio.Event()
        transport.release_configuration = asyncio.Event()
        reconciler.start()
        reconciler.apply(desired(1, "OldWifi"))
        await transport.configuration_started.wait()
        reconciler.apply(desired(2, "NewWifi"))
        transport.release_configuration.set()
        for _ in range(100):
            if reconciler.status.applied_generation == 2:
                break
            await asyncio.sleep(0.01)
        wifi_commands = [command for command, _ in transport.commands if "nmcli" in command]
        assert wifi_commands == ["sudo -n nmcli connection up NewWifi"]
        await reconciler.close()

    asyncio.run(asyncio.wait_for(run(), timeout=3))


def test_interrupted_artifact_download_is_transient(tmp_path: Path) -> None:
    class FailingTransport(FakeTransport):
        async def run(self, command: str, *, stdin=None, output=None, check=True):
            if command.startswith("test -f"):
                return CommandResult(("ssh", command), 1)
            raise CommandFailedError(CommandResult(("ssh", command), 75, stderr="download interrupted"))

    class FakeServer:
        def url(self):
            return "http://127.0.0.1/environment.sh"

    artifact_path = tmp_path / "environment.sh"
    artifact_path.write_text("artifact")
    reconciler = RobotReconciler(
        RobotTarget("nuc1", "127.0.0.1"),
        tmp_path,
        EnvironmentArtifact("key", artifact_path, "deadbeef"),
        FakeServer(),
        transport=FailingTransport(),
    )

    async def run() -> None:
        with pytest.raises(CommandFailedError) as error:
            await reconciler._ensure_environment()
        assert error.value.transient

    asyncio.run(run())


def test_connection_probe_retries_at_fixed_interval(tmp_path: Path) -> None:
    class ReconnectingTransport(FakeTransport):
        def __init__(self) -> None:
            super().__init__()
            self.attempts = 0

        async def connect(self) -> None:
            self.attempts += 1
            if self.attempts < 4:
                raise CommandFailedError(
                    CommandResult(("ssh",), 255, stderr="offline"),
                    transient=True,
                )

    transport = ReconnectingTransport()
    reconciler = RobotReconciler(
        RobotTarget("nuc1", "127.0.0.1"),
        tmp_path,
        None,
        None,
        retry_interval=0.01,
        transport=transport,
    )

    async def run() -> None:
        reconciler.start()
        for _ in range(20):
            if reconciler.status.connected:
                break
            await asyncio.sleep(0.01)
        assert reconciler.status.connected
        assert transport.attempts >= 4
        await reconciler.close()

    asyncio.run(asyncio.wait_for(run(), timeout=3))


def test_source_fingerprint_includes_dirty_file_contents(tmp_path: Path) -> None:
    subprocess.run(["git", "init", "-q"], cwd=tmp_path, check=True)
    subprocess.run(["git", "config", "user.email", "test@example.com"], cwd=tmp_path, check=True)
    subprocess.run(["git", "config", "user.name", "Test"], cwd=tmp_path, check=True)
    source = tmp_path / "source.txt"
    source.write_text("committed\n")
    subprocess.run(["git", "add", "source.txt"], cwd=tmp_path, check=True)
    subprocess.run(["git", "commit", "-qm", "initial"], cwd=tmp_path, check=True)

    source.write_text("first dirty value\n")
    first = source_fingerprint(tmp_path)
    source.write_text("second dirty value\n")
    second = source_fingerprint(tmp_path)

    assert first != second


def test_unchanged_generation_skips_configuration_and_wifi(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("deploy.reconcile.source_fingerprint", lambda _: "source")
    transport = FakeTransport()
    reconciler = RobotReconciler(
        RobotTarget("nuc1", "127.0.0.1"),
        tmp_path,
        None,
        None,
        retry_interval=0.01,
        transport=transport,
    )

    async def run() -> None:
        reconciler.start()
        reconciler.apply(desired(1))
        while reconciler.status.applied_generation != 1:
            await asyncio.sleep(0.01)
        first_count = len(transport.commands)
        reconciler.apply(desired(2))
        while reconciler.status.applied_generation != 2:
            await asyncio.sleep(0.01)
        second_commands = transport.commands[first_count:]
        assert not any(stdin for _, stdin in second_commands)
        assert not any("nmcli" in command for command, _ in second_commands)
        await reconciler.close()

    asyncio.run(asyncio.wait_for(run(), timeout=3))
