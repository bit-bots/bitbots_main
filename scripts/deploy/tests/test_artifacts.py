import hashlib
import json
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch

import yaml
from deploy.artifacts import ArtifactServer, ArtifactStore, EnvironmentArtifact


def artifact_directory(root: Path) -> Path:
    (root / "channel/linux-aarch64").mkdir(parents=True)
    (root / "channel/linux-aarch64/repodata.json").write_text("{}")
    (root / "pypi").mkdir()
    return root


def test_artifact_server_supports_resuming_package_downloads(tmp_path: Path) -> None:
    content = b"locked conda package"
    path = artifact_directory(tmp_path / "artifact")
    (path / "channel/linux-aarch64/package.conda").write_bytes(content)
    artifact = EnvironmentArtifact(
        key="key",
        path=path,
        sha256=hashlib.sha256(content).hexdigest(),
    )
    server = ArtifactServer(artifact, bind="127.0.0.1")
    server.start()
    try:
        request = urllib.request.Request(f"{server.base_url('127.0.0.1')}/conda/linux-aarch64/package.conda")
        request.add_header("Range", "bytes=7-")
        with urllib.request.urlopen(request) as response:
            assert response.status == 206
            assert response.read() == content[7:]
    finally:
        server.stop()


def test_artifact_server_flattens_locked_pypi_urls(tmp_path: Path) -> None:
    path = artifact_directory(tmp_path / "artifact")
    wheel = path / "pypi/example-1.0-py3-none-any.whl"
    wheel.write_bytes(b"wheel")
    server = ArtifactServer(EnvironmentArtifact("key", path, "digest"), bind="127.0.0.1")
    server.start()
    try:
        with urllib.request.urlopen(
            f"{server.base_url('127.0.0.1')}/pypi/aa/bb/example-1.0-py3-none-any.whl"
        ) as response:
            assert response.read() == b"wheel"
    finally:
        server.stop()


def test_artifact_server_rejects_unknown_token(tmp_path: Path) -> None:
    path = artifact_directory(tmp_path / "artifact")
    server = ArtifactServer(
        EnvironmentArtifact("key", path, "digest"),
        bind="127.0.0.1",
    )
    server.start()
    try:
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{server.port}/wrong/conda/repodata.json")
        except urllib.error.HTTPError as error:
            assert error.code == 404
        else:
            raise AssertionError("unknown token was accepted")
    finally:
        server.stop()


def test_pixi_config_redirects_locked_downloads_to_session_server(tmp_path: Path) -> None:
    path = artifact_directory(tmp_path / "artifact")
    artifact = EnvironmentArtifact(
        "key",
        path,
        "digest",
        channels=("https://conda.example/channel",),
        pypi_bases=("https://files.pythonhosted.org/packages",),
    )
    server = ArtifactServer(artifact, bind="127.0.0.1")
    server.start()
    try:
        config = server.pixi_config("127.0.0.1")
        assert '"https://conda.example/channel"' in config
        assert '"https://files.pythonhosted.org/packages"' in config
        assert f"http://127.0.0.1:{server.port}/{server.token}/conda" in config
        assert "downloads = 4" in config
        assert "solves = 1" in config
    finally:
        server.stop()


def test_prepare_creates_lock_keyed_offline_channel(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    (workspace / "scripts/deploy").mkdir(parents=True)
    (workspace / "scripts/deploy/pixi-pack-config.toml").write_text("[concurrency]\ndownloads = 4\n")
    (workspace / "pixi.lock").write_text(
        yaml.safe_dump(
            {
                "environments": {
                    "robot": {
                        "channels": [{"url": "https://conda.example/channel/"}],
                        "packages": {
                            "linux-aarch64": [
                                {"conda": "https://conda.example/channel/linux-aarch64/runtime.conda"},
                                {
                                    "pypi": (
                                        "https://files.pythonhosted.org/packages/aa/bb/example-1.0-py3-none-any.whl"
                                    )
                                },
                            ]
                        },
                    },
                }
            }
        )
    )
    cache = tmp_path / "cache"
    store = ArtifactStore(workspace, cache)

    def fake_run(command, cwd, check):
        output = Path(command[command.index("--output-file") + 1])
        artifact_directory(output)
        (output / "channel/linux-aarch64/runtime.conda").write_bytes(b"conda")
        (output / "pypi/example-1.0-py3-none-any.whl").write_bytes(b"wheel")

    with patch("deploy.artifacts.subprocess.run", side_effect=fake_run) as run:
        artifact = store.prepare()

    assert artifact.path.is_dir()
    assert "--directory-only" in run.call_args.args[0]
    assert "--create-executable" not in run.call_args.args[0]
    assert "linux-aarch64" in run.call_args.args[0]
    assert artifact.channels == ("https://conda.example/channel",)
    assert artifact.pypi_bases == ("https://files.pythonhosted.org/packages",)
    metadata = json.loads((cache / f"{artifact.key}.json").read_text())
    assert metadata["sha256"] == artifact.sha256
