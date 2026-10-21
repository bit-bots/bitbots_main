import importlib.util
from pathlib import Path
from unittest.mock import MagicMock

import yaml
from bitbots_mujoco_sim.simulation import Simulation
from game_controller_hsl_interfaces.msg import GameState


def test_domain_bridge_config_includes_gamestate(tmp_path):
    launch_file = (
        Path(__file__).resolve().parents[4]
        / "src"
        / "bitbots_misc"
        / "bitbots_bringup"
        / "launch"
        / "mujoco_simulation.launch.py"
    )
    spec = importlib.util.spec_from_file_location("mujoco_simulation_launch", str(launch_file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    config_path = module.generate_domain_bridge_config(11, tmp_path)
    with open(config_path) as f:
        config = yaml.safe_load(f)

    assert "gamestate" in config["topics"]
    assert config["topics"]["gamestate"]["type"] == "game_controller_hsl_interfaces/msg/GameState"
    assert "clock" in config["topics"]


def test_set_gamestate_updates_fields_and_publishes():
    sim = Simulation.__new__(Simulation)
    sim.gamestate_msg = GameState()
    sim.time_message = MagicMock()
    sim.gamestate_publisher = MagicMock()

    # Initial state
    assert sim.gamestate_msg.main_state == GameState.STATE_INITIAL
    assert not sim.gamestate_msg.stopped

    # Ready state
    sim.set_gamestate(main_state=GameState.STATE_READY, stopped=False)
    assert sim.gamestate_msg.main_state == GameState.STATE_READY
    assert not sim.gamestate_msg.stopped
    sim.gamestate_publisher.publish.assert_called_with(sim.gamestate_msg)

    # Set state
    sim.set_gamestate(main_state=GameState.STATE_SET, stopped=False)
    assert sim.gamestate_msg.main_state == GameState.STATE_SET
    assert not sim.gamestate_msg.stopped

    # Playing state
    sim.set_gamestate(main_state=GameState.STATE_PLAYING, stopped=False)
    assert sim.gamestate_msg.main_state == GameState.STATE_PLAYING
    assert not sim.gamestate_msg.stopped

    # Stopped state
    sim.set_gamestate(stopped=True)
    assert sim.gamestate_msg.main_state == GameState.STATE_PLAYING
    assert sim.gamestate_msg.stopped


def test_gamestate_gui_callbacks():
    sim = Simulation.__new__(Simulation)
    sim.gamestate_msg = GameState()
    sim.time_message = MagicMock()
    sim.gamestate_publisher = MagicMock()

    buttons = {}

    class MockGui:
        def add_folder(self, name):
            from contextlib import nullcontext

            return nullcontext()

        def add_button(self, name):
            btn = MagicMock()
            btn.on_click = lambda cb: buttons.update({name: cb})
            return btn

    server = MagicMock()
    server.gui = MockGui()

    sim._add_gamestate_gui(server)

    assert set(buttons.keys()) == {"Ready", "Set", "Playing", "Stopped"}

    # Trigger Ready button
    buttons["Ready"](None)
    assert sim.gamestate_msg.main_state == GameState.STATE_READY
    assert not sim.gamestate_msg.stopped

    # Trigger Set button
    buttons["Set"](None)
    assert sim.gamestate_msg.main_state == GameState.STATE_SET
    assert not sim.gamestate_msg.stopped

    # Trigger Playing button
    buttons["Playing"](None)
    assert sim.gamestate_msg.main_state == GameState.STATE_PLAYING
    assert not sim.gamestate_msg.stopped

    # Trigger Stopped button
    buttons["Stopped"](None)
    assert sim.gamestate_msg.stopped
