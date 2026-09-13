from unittest.mock import MagicMock

from manage.engine import DockerEngine, PodmanEngine
from manage.manager import ContainerManager


def test_manager_default_docker(monkeypatch):
    monkeypatch.setattr(ContainerManager, "execute_command", lambda self: None)
    cm = ContainerManager(["build-all"])
    assert isinstance(cm.engine, DockerEngine)


def test_manager_select_podman_cli(monkeypatch):
    monkeypatch.setattr(ContainerManager, "execute_command", lambda self: None)
    cm = ContainerManager(["-e", "podman", "build-all"])
    assert isinstance(cm.engine, PodmanEngine)


def test_manager_select_podman_env(monkeypatch):
    monkeypatch.setenv("CONTAINER_ENGINE", "podman")
    monkeypatch.setattr(ContainerManager, "execute_command", lambda self: None)
    cm = ContainerManager(["build-all"])
    assert isinstance(cm.engine, PodmanEngine)


def test_manager_dispatch_build_common(monkeypatch):
    mock_build_common = MagicMock()
    monkeypatch.setattr(ContainerManager, "build_common", mock_build_common)
    ContainerManager(["build-common"])
    mock_build_common.assert_called_once()


def test_manager_dispatch_run_project(monkeypatch):
    mock_run_project = MagicMock()
    monkeypatch.setattr(ContainerManager, "run_project", mock_run_project)
    ContainerManager(["run-project", "mickey"])
    mock_run_project.assert_called_once_with("mickey")


def test_manager_dispatch_run_target(monkeypatch):
    mock_run_target = MagicMock()
    monkeypatch.setattr(ContainerManager, "run_target", mock_run_target)
    ContainerManager(["run-target", "minnie"])
    mock_run_target.assert_called_once_with("minnie")


def test_manager_dispatch_stop_all(monkeypatch):
    mock_stop = MagicMock()
    monkeypatch.setattr(ContainerManager, "stop_all", mock_stop)
    ContainerManager(["stop-all"])
    mock_stop.assert_called_once()


def test_manager_dispatch_ssh(monkeypatch):
    mock_ssh = MagicMock()
    monkeypatch.setattr(ContainerManager, "ssh", mock_ssh)
    ContainerManager(["ssh", "mickey"])
    mock_ssh.assert_called_once_with("mickey")
