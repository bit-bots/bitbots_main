from pathlib import Path
from unittest.mock import MagicMock

import pytest
from manage.engine import DockerEngine, PodmanEngine, get_engine


def test_get_engine():
    assert isinstance(get_engine("docker"), DockerEngine)
    assert isinstance(get_engine("podman"), PodmanEngine)
    with pytest.raises(ValueError):
        get_engine("invalid_engine")


def test_docker_gpu_args(monkeypatch):
    engine = DockerEngine()
    monkeypatch.setattr("shutil.which", lambda x: "/usr/bin/nvidia-smi" if x == "nvidia-smi" else None)
    monkeypatch.setattr(Path, "is_dir", lambda self: str(self) == "/dev/dri")
    monkeypatch.setattr(Path, "exists", lambda self: False)

    args = engine.get_gpu_args()
    assert "--device" in args
    assert "/dev/dri" in args
    assert "--gpus" in args
    assert "all" in args


def test_podman_gpu_args(monkeypatch, tmp_path):
    engine = PodmanEngine()
    monkeypatch.setattr(Path, "is_dir", lambda self: str(self) == "/dev/dri")
    monkeypatch.setattr(Path, "exists", lambda self: False)

    args = engine.get_gpu_args()
    assert "--device" in args
    assert "/dev/dri" in args


def test_docker_build_image(monkeypatch):
    engine = DockerEngine()
    monkeypatch.setattr(engine, "check_available", lambda: True)
    mock_run = MagicMock()
    monkeypatch.setattr(engine, "run_cmd", mock_run)

    engine.build_image("test-img", "/path/Containerfile", "/path/ctx", build_args={"ARG1": "VAL1"})

    mock_run.assert_called_once_with(
        ["build", "-t", "test-img", "--build-arg", "ARG1=VAL1", "-f", "/path/Containerfile", "/path/ctx"],
        check=True,
    )


def test_docker_run_container(monkeypatch):
    engine = DockerEngine()
    monkeypatch.setattr(engine, "check_available", lambda: True)
    mock_run = MagicMock()
    monkeypatch.setattr(engine, "run_cmd", mock_run)

    engine.run_container(
        "test-img",
        "test-container",
        net_args=["--network", "bitbots-net"],
        env_args=["-e", "VAR=1"],
        gpu_args=["--device", "/dev/dri"],
        detached=True,
    )

    mock_run.assert_called_once_with(
        ["run", "-d", "--name", "test-container", "--network", "bitbots-net", "-e", "VAR=1", "--device", "/dev/dri", "test-img"],
        check=True,
    )


def test_stop_and_remove_containers(monkeypatch):
    engine = DockerEngine()
    monkeypatch.setattr(engine, "check_available", lambda: True)
    monkeypatch.setattr(engine, "find_containers", lambda pat: ["bitbots-project-run", "simulator"])
    mock_run = MagicMock()
    monkeypatch.setattr(engine, "run_cmd", mock_run)

    engine.stop_and_remove_containers()

    assert mock_run.call_count == 2
    mock_run.assert_any_call(["stop", "bitbots-project-run", "simulator"], check=False)
    mock_run.assert_any_call(["rm", "bitbots-project-run", "simulator"], check=False)


def test_podman_unsupported_swarm():
    engine = PodmanEngine()
    with pytest.raises(NotImplementedError):
        engine.swarm_init()
    with pytest.raises(NotImplementedError):
        engine.connect_host()
