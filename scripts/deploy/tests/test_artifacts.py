import hashlib
import json
import urllib.request
from pathlib import Path
from unittest.mock import patch

import yaml
from deploy.artifacts import ArtifactServer, ArtifactStore, EnvironmentArtifact


def test_artifact_server_supports_resuming_downloads(tmp_path: Path) -> None:
    content = b"portable robot environment"
    path = tmp_path / "robot.sh"
    path.write_bytes(content)
    artifact = EnvironmentArtifact(
        key="key",
        path=path,
        sha256=hashlib.sha256(content).hexdigest(),
    )
    server = ArtifactServer(artifact, bind="127.0.0.1")
    server.start()
    try:
        request = urllib.request.Request(server.url("127.0.0.1"))
        request.add_header("Range", "bytes=9-")
        with urllib.request.urlopen(request) as response:
            assert response.status == 206
            assert response.read() == content[9:]
    finally:
        server.stop()


def test_artifact_server_rejects_unknown_token(tmp_path: Path) -> None:
    path = tmp_path / "robot.sh"
    path.write_bytes(b"x")
    server = ArtifactServer(
        EnvironmentArtifact("key", path, hashlib.sha256(b"x").hexdigest()),
        bind="127.0.0.1",
    )
    server.start()
    try:
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{server.port}/wrong/robot.sh")
        except urllib.error.HTTPError as error:
            assert error.code == 404
        else:
            raise AssertionError("unknown token was accepted")
    finally:
        server.stop()


def test_prepare_creates_lock_keyed_executable(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    workspace.mkdir()
    (workspace / "scripts/deploy").mkdir(parents=True)
    (workspace / "scripts/deploy/robot_env.sh").write_text("activation-v1")
    (workspace / "pixi.lock").write_text(
        yaml.safe_dump(
            {
                "environments": {
                    "default": {"packages": ["laptop-only"]},
                    "robot": {"packages": ["robot-runtime"]},
                }
            }
        )
    )
    cache = tmp_path / "cache"
    store = ArtifactStore(workspace, cache)

    def fake_run(command, cwd, check):
        Path(command[command.index("--output-file") + 1]).write_bytes(b"executable")

    with patch("deploy.artifacts.subprocess.run", side_effect=fake_run) as run:
        artifact = store.prepare()

    assert artifact.path.read_bytes() == b"executable"
    assert "--create-executable" in run.call_args.args[0]
    assert "linux-aarch64" in run.call_args.args[0]
    assert json.loads((cache / f"{artifact.key}.json").read_text())["sha256"] == artifact.sha256
