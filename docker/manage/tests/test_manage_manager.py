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


def test_manager_dispatch_build_base(monkeypatch):
    mock_build_base = MagicMock()
    monkeypatch.setattr(ContainerManager, "build_base", mock_build_base)
    ContainerManager(["build-base"])
    mock_build_base.assert_called_once()


def test_manager_build_methods(monkeypatch, tmp_path):
    monkeypatch.setattr(ContainerManager, "execute_command", lambda self: None)
    cm = ContainerManager(["build-all"])
    mock_build_image = MagicMock()
    cm.engine.build_image = mock_build_image

    # test build_base
    cm.build_base()
    mock_build_image.assert_called_with(
        "bitbots-base", cm.engine.build_image.call_args[0][1], cm.engine.build_image.call_args[0][2]
    )

    # test build_project with SSH key
    key_file = tmp_path / "id_ed25519.pub"
    key_file.write_text("ssh-ed25519 AAAAC3NzaC1lZDI1NTE5 test")
    monkeypatch.setattr("manage.manager.find_ssh_key", lambda: key_file)
    mock_build_image.reset_mock()
    cm.build_project()
    mock_build_image.assert_called_with(
        "bitbots-project",
        cm.engine.build_image.call_args[0][1],
        cm.engine.build_image.call_args[0][2],
        build_args={"BASE_IMAGE": "bitbots-base", "ssh_pub_key": "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5 test"},
    )

    # test build_all
    mock_base = MagicMock()
    mock_proj = MagicMock()
    cm.build_base = mock_base
    cm.build_project = mock_proj
    cm.build_all()
    mock_base.assert_called_once()
    mock_proj.assert_called_once()


def test_manager_dispatch_run_project(monkeypatch):
    mock_run_project = MagicMock()
    monkeypatch.setattr(ContainerManager, "run_project", mock_run_project)
    ContainerManager(["run-project", "mickey"])
    mock_run_project.assert_called_once_with("mickey", zenoh_router=False)


def test_manager_dispatch_run_project_zenoh(monkeypatch):
    mock_run_project = MagicMock()
    monkeypatch.setattr(ContainerManager, "run_project", mock_run_project)
    ContainerManager(["run-project", "mickey", "--zenoh-router"])
    mock_run_project.assert_called_once_with("mickey", zenoh_router=True)


def test_manager_dispatch_run_simulator(monkeypatch):
    mock_run_simulator = MagicMock()
    monkeypatch.setattr(ContainerManager, "run_simulator", mock_run_simulator)
    ContainerManager(["run-simulator"])
    mock_run_simulator.assert_called_once_with(None, zenoh_router=False)


def test_manager_dispatch_run_simulator_zenoh(monkeypatch):
    mock_run_simulator = MagicMock()
    monkeypatch.setattr(ContainerManager, "run_simulator", mock_run_simulator)
    ContainerManager(["run-simulator", "--zenoh"])
    mock_run_simulator.assert_called_once_with(None, zenoh_router=True)


def test_manager_dispatch_run_type_project(monkeypatch):
    mock_run_project = MagicMock()
    monkeypatch.setattr(ContainerManager, "run_project", mock_run_project)
    ContainerManager(["run", "project", "mickey", "--zenoh-router"])
    mock_run_project.assert_called_once_with("mickey", zenoh_router=True)


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


def test_manager_run_project_env_zenoh(monkeypatch):
    monkeypatch.setattr(ContainerManager, "execute_command", lambda self: None)
    cm = ContainerManager(["run-project"])
    monkeypatch.setattr(cm, "create_network", lambda subnet=None: None)
    mock_run_container = MagicMock()
    cm.engine.run_container = mock_run_container

    # Without Zenoh
    cm.run_project(zenoh_router=False)
    env_args = mock_run_container.call_args[0][3]
    assert "-e" not in env_args or "START_ZENOH_ROUTER=1" not in env_args

    # With Zenoh
    mock_run_container.reset_mock()
    cm.run_project(zenoh_router=True)
    env_args = mock_run_container.call_args[0][3]
    assert "START_ZENOH_ROUTER=1" in env_args


def test_manager_run_simulator_env_zenoh(monkeypatch):
    monkeypatch.setattr(ContainerManager, "execute_command", lambda self: None)
    cm = ContainerManager(["run-simulator"])
    monkeypatch.setattr(cm, "create_network", lambda subnet=None: None)
    mock_run_container = MagicMock()
    cm.engine.run_container = mock_run_container

    # Without Zenoh
    cm.run_simulator(zenoh_router=False)
    env_args = mock_run_container.call_args[0][3]
    assert "START_ZENOH_ROUTER=1" not in env_args
    assert "ZENOH_MODE=router" not in env_args
    assert "SIMULATOR=1" in env_args

    # With Zenoh
    mock_run_container.reset_mock()
    cm.run_simulator(zenoh_router=True)
    env_args = mock_run_container.call_args[0][3]
    assert "START_ZENOH_ROUTER=1" in env_args
    assert "ZENOH_MODE=router" in env_args
    assert "SIMULATOR=1" in env_args
