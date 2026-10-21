#!/usr/bin/env python3

import json
import math
import signal
import threading
import time
from pathlib import Path
from typing import cast

import pytest
from bitbots_body_behavior import blockly_server, body_behavior, simple_behavior, simple_behavior_web_wrapper
from rclpy.node import Node

from bitbots_msgs.msg import HeadMode


class DummyLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message: str):
        self.infos.append(message)

    def warn(self, _: str):
        self.warnings.append(_)

    def error(self, message: str):
        self.errors.append(message)


class DummyDuration:
    def __init__(self, nanoseconds: int = 1_000_000_000):
        self.nanoseconds = nanoseconds


class DummyClockNow:
    def __init__(self, nanoseconds: int = 1_000_000_000):
        self.nanoseconds = nanoseconds

    @staticmethod
    def to_msg():
        return None

    def __sub__(self, other):
        other_ns = getattr(other, "nanoseconds", 0)
        return DummyDuration(self.nanoseconds - other_ns)


class DummyClock:
    def __init__(self):
        self.slept_durations = []
        self._time_ns = 1_000_000_000

    def now(self):
        self._time_ns += 300_000_000
        return DummyClockNow(self._time_ns)

    def sleep_for(self, duration):
        self.slept_durations.append(duration)
        if duration.nanoseconds > 0:
            time.sleep(min(duration.nanoseconds / 1e9, 0.05))
        return True


class DummyNode:
    def __init__(self, parameters=None):
        self.logger = DummyLogger()
        self.parameters = parameters or {}
        self.clock = DummyClock()

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger

    def get_parameter(self, name):
        class Parameter:
            def __init__(self, value):
                self.value = value

        return Parameter(self.parameters.get(name, ""))


class DummyPathfinding:
    def __init__(self):
        self.last_goal = None
        self.stopped = False

    def publish(self, goal):
        self.last_goal = goal

    def stop_walk(self):
        self.stopped = True


class DummyKick:
    def __init__(self, kick_duration=0.0):
        self.last_kick = None
        self.is_currently_kicking = False
        self._kick_duration = kick_duration
        self._kick_timer = None

    def start_rl_kick(self, direction_deg, strength):
        self.last_kick = (direction_deg, strength)
        self.is_currently_kicking = True
        if self._kick_duration <= 0.0:
            self.is_currently_kicking = False
        else:
            self._kick_timer = threading.Timer(self._kick_duration, self._finish_kick)
            self._kick_timer.daemon = True
            self._kick_timer.start()
        return True

    def _finish_kick(self):
        self.is_currently_kicking = False


class DummyMisc:
    def __init__(self):
        self.head_mode = None

    def set_head_duty(self, head_mode):
        self.head_mode = head_mode


class DummyWorldModel:
    map_frame = "map"

    @staticmethod
    def get_current_position():
        return 1.0, 2.0, 3.0

    @staticmethod
    def get_ball_position_xy():
        return 4.0, 5.0

    @staticmethod
    def ball_has_been_seen():
        return True

    @staticmethod
    def forget_ball():
        pass


class DummyGamestate:
    def __init__(self):
        self.main_state = 3  # GameState.STATE_PLAYING
        self.stopped = False
        self.has_kick_val = True
        self.seconds_remaining = 500.0
        self.secondary_seconds_remaining = 30.0
        self.own_score = 2
        self.rival_score = 1
        self.penalized = False

    def get_main_state(self):
        return self.main_state

    def is_stopped(self):
        return self.stopped

    def has_kick(self):
        return self.has_kick_val

    def get_seconds_remaining(self):
        return self.seconds_remaining

    def get_secondary_seconds_remaining(self):
        return self.secondary_seconds_remaining

    def get_own_score(self):
        return self.own_score

    def get_rival_score(self):
        return self.rival_score

    def get_is_penalized(self):
        return self.penalized


class DummyBlackboard:
    def __init__(self):
        self.world_model = DummyWorldModel()
        self.pathfinding = DummyPathfinding()
        self.kick = DummyKick()
        self.misc = DummyMisc()
        self.gamestate = DummyGamestate()


class DummyTeamData:
    def __init__(self):
        self.strategy_published = 0
        self.time_to_ball_published = 0

    def publish_strategy(self):
        self.strategy_published += 1

    def publish_time_to_ball(self):
        self.time_to_ball_published += 1


class DummyPathfindingWithTimer:
    def __init__(self):
        self.time_to_ball_calculated = 0

    def calculate_time_to_ball(self):
        self.time_to_ball_calculated += 1


class DummyBlackboardForBehavior:
    def __init__(self):
        self.world_model = DummyWorldModel()
        self.pathfinding = DummyPathfindingWithTimer()
        self.team_data = DummyTeamData()
        self.gamestate = DummyGamestate()
        self.config = {"time_to_ball_divider": 2}
        self.cache_cleared = 0

    def clear_cache(self):
        self.cache_cleared += 1


def create_simple_behavior_for_test(main_function, node=None, blackboard=None):
    if node is None:
        node = DummyNode()
    if blackboard is None:
        blackboard = DummyBlackboardForBehavior()

    behavior = object.__new__(body_behavior.BodySimpleBehavior)
    behavior.counter = 0
    behavior.node = node
    behavior.blackboard = blackboard
    behavior._main_function = main_function
    behavior._main_thread = None
    behavior._state_lock = threading.Lock()
    behavior._thread_state = "not_started"
    behavior._thread_failure_traceback = None
    behavior._thread_failure_supervised = False
    return behavior, node, blackboard


def test_simple_behavior_api_forwards_to_runtime(monkeypatch):
    runtime = simple_behavior.SimpleBehaviorRuntime(DummyNode(), DummyBlackboard())
    monkeypatch.setattr(simple_behavior, "_runtime", runtime)

    simple_behavior.set_goal_pose(0.1, 0.2, 15.0)
    assert runtime._blackboard.pathfinding.last_goal is not None
    assert runtime._blackboard.pathfinding.last_goal.pose.position.x == 0.1
    assert runtime._blackboard.pathfinding.last_goal.pose.position.y == 0.2

    assert simple_behavior.kick_ball(45.0, 2.5)
    assert runtime._blackboard.kick.last_kick == (45.0, 2.5)

    simple_behavior.set_head_mode(simple_behavior.HEAD_MODE_TRACK_BALL)
    assert runtime._blackboard.misc.head_mode == HeadMode.TRACK_BALL

    simple_behavior.stop()
    assert runtime._blackboard.pathfinding.stopped

    current_pose = simple_behavior.get_current_pose()
    assert current_pose == pytest.approx((1.0, 2.0, math.degrees(3.0)))
    assert current_pose is not None
    assert current_pose.x == 1.0
    assert current_pose.y == 2.0
    assert current_pose.yew_deg == pytest.approx(math.degrees(3.0))

    ball_pose = simple_behavior.get_ball_pose()
    assert ball_pose == (4.0, 5.0)
    assert ball_pose is not None
    assert ball_pose.x == 4.0
    assert ball_pose.y == 5.0

    simple_behavior.sleep(0.1)
    simple_behavior.sleep(0.5)
    clock = runtime._node.get_clock()
    assert [duration.nanoseconds for duration in clock.slept_durations] == [100_000_000, 500_000_000]


def test_simple_behavior_sleep_ignores_non_positive_duration(monkeypatch):
    runtime = simple_behavior.SimpleBehaviorRuntime(DummyNode(), DummyBlackboard())
    monkeypatch.setattr(simple_behavior, "_runtime", runtime)

    simple_behavior.sleep(0)
    simple_behavior.sleep(-1.5)
    assert len(runtime._node.get_clock().slept_durations) == 0


def test_simple_behavior_sleep_with_real_node():
    import rclpy
    from rclpy.node import Node as RclpyNode

    if not rclpy.ok():
        rclpy.init()
    node = RclpyNode("test_simple_behavior_sleep_node")
    try:
        runtime = simple_behavior.SimpleBehaviorRuntime(node, DummyBlackboard())
        simple_behavior.configure_runtime(runtime)
        start = time.monotonic()
        simple_behavior.sleep(0.05)
        elapsed = time.monotonic() - start
        assert elapsed >= 0.04
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_get_current_pose_raises_when_pose_is_unavailable(monkeypatch):
    runtime = simple_behavior.SimpleBehaviorRuntime(DummyNode(), DummyBlackboard())
    monkeypatch.setattr(runtime._blackboard.world_model, "get_current_position", lambda: None)
    monkeypatch.setattr(simple_behavior, "_runtime", runtime)

    with pytest.raises(RuntimeError, match="Current robot pose is not available"):
        simple_behavior.get_current_pose()


def test_get_ball_pose_returns_none_below_confidence_threshold(monkeypatch):
    runtime = simple_behavior.SimpleBehaviorRuntime(DummyNode(), DummyBlackboard())
    monkeypatch.setattr(runtime._blackboard.world_model, "ball_has_been_seen", lambda: False)
    monkeypatch.setattr(simple_behavior, "_runtime", runtime)

    assert simple_behavior.get_ball_pose() is None


def test_pose_math_helpers():
    origin = simple_behavior.make_pose(0, 0)
    target = simple_behavior.make_pose(3, 4)
    current_pose = simple_behavior.CurrentPose(1.0, 2.0, 0.5)

    assert origin == simple_behavior.BallPose(0.0, 0.0)
    assert simple_behavior.pose_distance(origin, target) == pytest.approx(5.0)
    assert simple_behavior.pose_add(current_pose, target) == simple_behavior.BallPose(4.0, 6.0)
    assert simple_behavior.pose_subtract(target, current_pose) == simple_behavior.BallPose(2.0, 2.0)
    assert simple_behavior.pose_direction(origin, target) == pytest.approx(53.130102)
    assert simple_behavior.pose_from_direction(origin, 2, 90) == pytest.approx(simple_behavior.BallPose(0.0, 2.0))


def test_set_goal_pose_from_pose_forwards_coordinates(monkeypatch):
    runtime = simple_behavior.SimpleBehaviorRuntime(DummyNode(), DummyBlackboard())
    monkeypatch.setattr(simple_behavior, "_runtime", runtime)

    simple_behavior.set_goal_pose_from_pose(simple_behavior.make_pose(1.5, -0.5))

    goal = runtime._blackboard.pathfinding.last_goal
    assert goal.pose.position.x == 1.5
    assert goal.pose.position.y == -0.5


def test_game_state_queries(monkeypatch):
    runtime = simple_behavior.SimpleBehaviorRuntime(DummyNode(), DummyBlackboard())
    monkeypatch.setattr(simple_behavior, "_runtime", runtime)

    # Defaults from DummyGamestate
    assert simple_behavior.get_game_state() == simple_behavior.GAME_STATE_PLAYING
    assert simple_behavior.is_game_state(simple_behavior.GAME_STATE_PLAYING)
    assert not simple_behavior.is_game_state(simple_behavior.GAME_STATE_READY)
    assert not simple_behavior.is_game_stopped()
    assert not simple_behavior.is_stopped()
    assert simple_behavior.has_kick()
    assert simple_behavior.get_seconds_remaining() == 500.0
    assert simple_behavior.get_secondary_seconds_remaining() == 30.0
    assert simple_behavior.get_own_score() == 2
    assert simple_behavior.get_rival_score() == 1
    assert not simple_behavior.is_penalized()

    # Change gamestate fields
    runtime._blackboard.gamestate.main_state = 1  # READY
    assert simple_behavior.get_game_state() == simple_behavior.GAME_STATE_READY
    assert simple_behavior.is_game_state(simple_behavior.GAME_STATE_READY)

    runtime._blackboard.gamestate.main_state = 2  # SET
    assert simple_behavior.get_game_state() == simple_behavior.GAME_STATE_SET

    runtime._blackboard.gamestate.main_state = 0  # INITIAL
    assert simple_behavior.get_game_state() == simple_behavior.GAME_STATE_INITIAL

    runtime._blackboard.gamestate.main_state = 4  # FINISHED
    assert simple_behavior.get_game_state() == simple_behavior.GAME_STATE_FINISHED

    runtime._blackboard.gamestate.main_state = 99  # Unknown
    assert simple_behavior.get_game_state() == "UNKNOWN"

    runtime._blackboard.gamestate.stopped = True
    assert simple_behavior.is_game_stopped()
    assert simple_behavior.is_stopped()

    runtime._blackboard.gamestate.has_kick_val = False
    assert not simple_behavior.has_kick()

    runtime._blackboard.gamestate.penalized = True
    assert simple_behavior.is_penalized()


def test_simple_behavior_requires_runtime(monkeypatch):
    monkeypatch.setattr(simple_behavior, "_runtime", None)

    with pytest.raises(RuntimeError):
        simple_behavior.get_current_pose()

    with pytest.raises(RuntimeError):
        simple_behavior.sleep(0.1)


def test_kick_ball_waits_for_kick_to_finish(monkeypatch):
    blackboard = DummyBlackboard()
    blackboard.kick = DummyKick(kick_duration=0.05)
    runtime = simple_behavior.SimpleBehaviorRuntime(DummyNode(), blackboard)
    monkeypatch.setattr(simple_behavior, "_runtime", runtime)

    start = time.monotonic()
    assert simple_behavior.kick_ball(0.0)
    elapsed = time.monotonic() - start

    assert elapsed >= 0.04


def test_simple_behavior_main_thread_starts_once():
    started_event = threading.Event()
    finished_event = threading.Event()
    run_counter = 0

    def user_main():
        nonlocal run_counter
        run_counter += 1
        started_event.set()
        time.sleep(0.05)
        finished_event.set()

    behavior, _, _ = create_simple_behavior_for_test(user_main)

    behavior._run_behavior_step()
    assert started_event.wait(timeout=0.5)

    behavior._run_behavior_step()
    assert finished_event.wait(timeout=0.5)
    assert run_counter == 1


def test_simple_behavior_loop_stays_non_blocking_with_sleeping_main():
    started_event = threading.Event()
    release_event = threading.Event()

    def user_main():
        started_event.set()
        release_event.wait(timeout=1.0)

    blackboard = DummyBlackboardForBehavior()
    behavior, _, _ = create_simple_behavior_for_test(user_main, blackboard=blackboard)

    start_time = time.monotonic()
    behavior.loop()
    elapsed = time.monotonic() - start_time

    assert elapsed < 0.1
    assert started_event.is_set()
    assert blackboard.team_data.strategy_published == 1
    assert blackboard.team_data.time_to_ball_published == 1
    assert blackboard.cache_cleared == 1
    release_event.set()


def test_simple_behavior_thread_exception_is_surfacable():
    def user_main():
        raise RuntimeError("boom")

    node = DummyNode()
    behavior, _, _ = create_simple_behavior_for_test(user_main, node=node)
    behavior._run_behavior_step()
    time.sleep(0.05)

    behavior._run_behavior_step()

    assert any("failed" in message for message in node.logger.errors)
    assert any("RuntimeError: boom" in message for message in node.logger.errors)


def test_simple_behavior_shutdown_join_is_bounded():
    release_event = threading.Event()

    def user_main():
        release_event.wait(timeout=5.0)

    node = DummyNode()
    behavior, _, _ = create_simple_behavior_for_test(user_main, node=node)
    behavior._run_behavior_step()
    time.sleep(0.05)

    start_time = time.monotonic()
    behavior.shutdown()
    elapsed = time.monotonic() - start_time

    assert elapsed < 1.0
    assert any("still running" in message for message in node.logger.warnings)
    release_event.set()


def test_submit_script_accepts_and_runs_to_completion():
    behavior, _, _ = create_simple_behavior_for_test(lambda: None)

    accepted, message = behavior.submit_script("def main():\n    pass\n")
    assert accepted
    assert "accepted" in message.lower()

    behavior._run_behavior_step()
    time.sleep(0.05)

    assert behavior.get_thread_state() == "finished"


def test_submit_script_rejects_while_main_is_running():
    release_event = threading.Event()

    def user_main():
        release_event.wait(timeout=1.0)

    behavior, _, _ = create_simple_behavior_for_test(user_main)
    behavior._run_behavior_step()
    time.sleep(0.05)

    accepted, message = behavior.submit_script("def main():\n    pass\n")
    release_event.set()

    assert not accepted
    assert "already running" in message


def test_simple_behavior_process_manager_restarts_running_process(monkeypatch, tmp_path):
    class FakeProcess:
        next_pid = 100

        def __init__(self):
            self.pid = FakeProcess.next_pid
            FakeProcess.next_pid += 1
            self.terminated = False
            self.killed = False
            self.return_code = None

        def poll(self):
            return self.return_code

        def terminate(self):
            self.terminated = True
            self.return_code = -15

        def kill(self):
            self.killed = True
            self.return_code = -9

        def wait(self, timeout=None):
            return self.return_code

    started_processes = []
    started_commands = []
    started_in_new_sessions = []
    process_group_signals = []

    def fake_popen(command, start_new_session, text, env):
        del text, env
        started_commands.append(command)
        started_in_new_sessions.append(start_new_session)
        process = FakeProcess()
        started_processes.append(process)
        return process

    def fake_killpg(process_group_id, sent_signal):
        process = next(process for process in started_processes if process.pid == process_group_id)
        if sent_signal == 0:
            if process.return_code is not None:
                raise ProcessLookupError
            return
        process_group_signals.append((process_group_id, sent_signal))
        process.return_code = -sent_signal

    monkeypatch.setattr(simple_behavior_web_wrapper.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(simple_behavior_web_wrapper.os, "killpg", fake_killpg)
    monkeypatch.setattr(
        simple_behavior_web_wrapper,
        "get_package_share_directory",
        lambda package_name: f"/share/{package_name}",
        raising=False,
    )

    manager = simple_behavior_web_wrapper.SimpleBehaviorProcessManager(
        cast(
            Node,
            DummyNode(
                {
                    "dsd_file": "custom.dsd",
                    "behavior_mode": "simple_web",
                    "simple_behavior_script": "/ignored/launch-script.py",
                    "odom_frame": "robot/odom",
                    "map_frame": "robot/map",
                    "base_footprint_frame": "robot/base_footprint",
                    "use_sim_time": True,
                }
            ),
        )
    )
    manager._script_path = tmp_path / "simple_behavior_script.py"

    accepted, _ = manager.submit_script("def main():\n    pass\n", "first.py")
    assert accepted
    assert len(started_processes) == 1
    assert started_in_new_sessions == [True]
    assert started_commands[0] == [
        "ros2",
        "run",
        "bitbots_body_behavior",
        "body_behavior",
        "--ros-args",
        "--params-file",
        "/share/bitbots_body_behavior/config/body_behavior.yaml",
        "--params-file",
        "/share/bitbots_body_behavior/config/animations.yaml",
        "-p",
        "dsd_file:=custom.dsd",
        "-p",
        "behavior_mode:=simple_web",
        "-p",
        "simple_behavior_script:=/ignored/launch-script.py",
        "-p",
        "odom_frame:=robot/odom",
        "-p",
        "map_frame:=robot/map",
        "-p",
        "base_footprint_frame:=robot/base_footprint",
        "-p",
        "use_sim_time:=true",
        "-p",
        "behavior_mode:=simple",
        "-p",
        f"simple_behavior_script:={manager._script_path}",
    ]
    first_process = started_processes[0]
    assert manager.get_state() == "running"

    accepted, _ = manager.submit_script("def main():\n    pass\n", "second.py")
    assert accepted
    assert len(started_processes) == 2
    assert process_group_signals == [(first_process.pid, signal.SIGTERM)]
    assert manager.get_state() == "running"

    manager.shutdown()
    assert process_group_signals == [
        (first_process.pid, signal.SIGTERM),
        (started_processes[1].pid, signal.SIGTERM),
    ]


def test_simple_behavior_process_manager_kills_unresponsive_process_group(monkeypatch):
    class UnresponsiveProcess:
        pid = 200
        return_code = None

        def poll(self):
            return self.return_code

        def wait(self, timeout=None):
            if self.return_code is None:
                raise simple_behavior_web_wrapper.subprocess.TimeoutExpired("body_behavior", timeout)
            return self.return_code

    sent_signals = []
    process = UnresponsiveProcess()

    def fake_killpg(process_group_id, sent_signal):
        sent_signals.append((process_group_id, sent_signal))
        if sent_signal == signal.SIGKILL:
            process.return_code = -signal.SIGKILL

    monkeypatch.setattr(simple_behavior_web_wrapper.os, "killpg", fake_killpg)

    manager = simple_behavior_web_wrapper.SimpleBehaviorProcessManager(
        cast(
            Node,
            DummyNode(
                {
                    "dsd_file": "behavior.dsd",
                    "behavior_mode": "simple_web",
                    "simple_behavior_script": "",
                    "odom_frame": "odom",
                    "map_frame": "map",
                    "base_footprint_frame": "base_footprint",
                    "use_sim_time": False,
                }
            ),
        )
    )
    manager._STOP_TIMEOUT_SECONDS = 0.0
    manager._process = process

    manager.shutdown()

    assert sent_signals == [
        (process.pid, signal.SIGTERM),
        (process.pid, signal.SIGKILL),
    ]


@pytest.mark.parametrize(
    ("configured_port", "ros_domain_id", "expected_port"),
    [
        (8123, "42", 8123),
        (0, "42", 7542),
        (0, None, 7500),
    ],
)
def test_simple_web_wrapper_resolves_web_port(monkeypatch, configured_port, ros_domain_id, expected_port):
    if ros_domain_id is None:
        monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    else:
        monkeypatch.setenv("ROS_DOMAIN_ID", ros_domain_id)

    assert simple_behavior_web_wrapper.SimpleBehaviorWebWrapper._resolve_web_port(configured_port) == expected_port


def test_simple_web_wrapper_starts_and_stops_http_server(monkeypatch):
    class FakeServer:
        def __init__(self, address, handler):
            self.address = address
            self.handler = handler
            self.daemon_threads = False
            self.shutdown_called = False
            self.server_close_called = False
            self.serve_forever_calls = 0

        def serve_forever(self, poll_interval=0.5):
            self.serve_forever_calls += 1

        def shutdown(self):
            self.shutdown_called = True

        def server_close(self):
            self.server_close_called = True

    monkeypatch.setattr(simple_behavior_web_wrapper, "ThreadingHTTPServer", FakeServer)

    wrapper = object.__new__(simple_behavior_web_wrapper.SimpleBehaviorWebWrapper)
    wrapper._node = cast(
        Node, DummyNode(parameters={"simple_behavior_web_host": "127.0.0.1", "simple_behavior_web_port": 8765})
    )
    wrapper._process_manager = simple_behavior_web_wrapper.SimpleBehaviorProcessManager(wrapper._node)
    wrapper._web_host = "127.0.0.1"
    wrapper._web_port = 8765
    wrapper._http_server = None
    wrapper._http_server_thread = None

    wrapper._start_web_server()

    assert wrapper._http_server is not None
    fake_server = cast(FakeServer, wrapper._http_server)
    assert fake_server.address == ("127.0.0.1", 8765)
    time.sleep(0.05)
    assert fake_server.serve_forever_calls >= 1

    wrapper.shutdown()
    assert fake_server.shutdown_called
    assert fake_server.server_close_called


def test_opponents_basic_script():
    from pathlib import Path

    script_path = Path(__file__).parents[4] / "opponents" / "basic.py"
    assert script_path.is_file()

    code = script_path.read_text()
    behavior, _, _ = create_simple_behavior_for_test(lambda: None)
    accepted, message = behavior.submit_script(code)
    assert accepted, f"Script rejected: {message}"


def test_opponents_advanced_script():
    from pathlib import Path

    script_path = Path(__file__).parents[4] / "opponents" / "advanced.py"
    assert script_path.is_file()

    code = script_path.read_text()
    behavior, _, _ = create_simple_behavior_for_test(lambda: None)
    accepted, message = behavior.submit_script(code)
    assert accepted, f"Script rejected: {message}"


def test_blockly_server_resolves_designs_directory(tmp_path):
    custom_dir = tmp_path / "custom_designs"
    resolved = blockly_server.BlocklyDesignStore._resolve_designs_directory(custom_dir)
    assert resolved == custom_dir.resolve()
    assert custom_dir.is_dir()

    default_resolved = blockly_server.BlocklyDesignStore._resolve_designs_directory(None)
    assert default_resolved == (Path.home() / ".bitbots" / "simple_behavior_designs").resolve()


def test_blockly_server_sanitizes_design_name():
    sanitize = blockly_server.BlocklyDesignStore.sanitize_design_name
    assert sanitize("striker") == "striker"
    assert sanitize("striker.json") == "striker"
    assert sanitize("striker.xml") == "striker"
    assert sanitize("  pass_and_kick  ") == "pass_and_kick"

    with pytest.raises(ValueError):
        sanitize("")
    with pytest.raises(ValueError):
        sanitize("   ")
    with pytest.raises(ValueError):
        sanitize("../etc/passwd")
    with pytest.raises(ValueError):
        sanitize("subdir/design")
    with pytest.raises(ValueError):
        sanitize("subdir\\design")
    with pytest.raises(ValueError):
        sanitize(".hidden")


def test_blockly_design_store_crud(tmp_path):
    designs_dir = tmp_path / "designs"
    store = blockly_server.BlocklyDesignStore(designs_dir)

    assert store.list_designs() == []

    saved_name = store.save_design(
        name="test_design",
        xml_content="<xml><block type='main'/></xml>",
        script_content="def main(): pass",
        metadata={"author": "bitbots"},
    )
    assert saved_name == "test_design"

    designs = store.list_designs()
    assert len(designs) == 1
    assert designs[0]["name"] == "test_design"
    assert designs[0]["filename"] == "test_design.json"

    loaded = store.load_design("test_design")
    assert loaded["ok"] is True
    assert loaded["name"] == "test_design"
    assert loaded["xml"] == "<xml><block type='main'/></xml>"
    assert loaded["script"] == "def main(): pass"
    assert loaded["metadata"] == {"author": "bitbots"}

    # Test loading a legacy / raw .xml design file
    raw_xml_file = designs_dir / "legacy_play.xml"
    raw_xml_file.write_text("<xml><block type='legacy'/></xml>", encoding="utf-8")

    designs = store.list_designs()
    assert len(designs) == 2
    design_names = [d["name"] for d in designs]
    assert "test_design" in design_names
    assert "legacy_play" in design_names

    legacy_loaded = store.load_design("legacy_play")
    assert legacy_loaded["name"] == "legacy_play"
    assert legacy_loaded["xml"] == "<xml><block type='legacy'/></xml>"

    # Test deleting
    assert store.delete_design("test_design") is True
    assert store.delete_design("test_design") is False
    with pytest.raises(FileNotFoundError):
        store.load_design("test_design")

    assert store.delete_design("legacy_play") is True
    assert store.list_designs() == []


def test_blockly_server_http_endpoints(tmp_path):
    import urllib.error
    import urllib.request

    designs_dir = tmp_path / "designs_http"
    static_dir = Path(__file__).resolve().parents[1] / "docs" / "blockly"

    server = blockly_server.create_blockly_server(
        host="127.0.0.1",
        port=0,
        directory=static_dir,
        designs_directory=designs_dir,
    )
    server_thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
    server_thread.start()

    port = server.server_address[1]
    base_url = f"http://127.0.0.1:{port}"

    try:
        # 1. Health check
        req = urllib.request.Request(f"{base_url}/health")
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode())
            assert data["ok"] is True
            assert data["service"] == "blockly_server"

        # 2. Static file serving (HTML)
        req = urllib.request.Request(f"{base_url}/simple_behavior_blockly.html")
        with urllib.request.urlopen(req) as resp:
            content = resp.read().decode()
            assert "<!doctype html>" in content.lower()
            assert resp.headers.get("Access-Control-Allow-Origin") == "*"

        # 3. List designs when empty
        req = urllib.request.Request(f"{base_url}/designs")
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode())
            assert data["ok"] is True
            assert data["designs"] == []

        # 4. Save design via POST /designs
        save_payload = json.dumps(
            {
                "name": "striker",
                "xml": "<xml><block type='kick'/></xml>",
                "script": "def main(): kick_ball(0)",
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            f"{base_url}/designs", data=save_payload, headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode())
            assert data["ok"] is True
            assert data["name"] == "striker"

        # 5. Load design via GET /designs/striker
        req = urllib.request.Request(f"{base_url}/designs/striker")
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode())
            assert data["ok"] is True
            assert data["name"] == "striker"
            assert "<block type='kick'/>" in data["xml"]

        # 6. Load design via query param GET /designs?name=striker
        req = urllib.request.Request(f"{base_url}/designs?name=striker")
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode())
            assert data["ok"] is True
            assert data["name"] == "striker"

        # 7. Delete design via DELETE /designs/striker
        req = urllib.request.Request(f"{base_url}/designs/striker", method="DELETE")
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode())
            assert data["ok"] is True

        # 8. Loading deleted design returns 404
        req = urllib.request.Request(f"{base_url}/designs/striker")
        with pytest.raises(urllib.error.HTTPError) as exc_info:
            urllib.request.urlopen(req)
        assert exc_info.value.code == 404

        # 9. Invalid design name returns 400
        bad_payload = json.dumps({"name": "../escape", "xml": "<xml/>"}).encode("utf-8")
        req = urllib.request.Request(
            f"{base_url}/designs", data=bad_payload, headers={"Content-Type": "application/json"}
        )
        with pytest.raises(urllib.error.HTTPError) as exc_info:
            urllib.request.urlopen(req)
        assert exc_info.value.code == 400

    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=1.0)


def test_simple_behavior_web_wrapper_http_endpoints(monkeypatch):
    import urllib.error
    import urllib.request

    dummy_node = DummyNode(
        {
            "simple_behavior_web_host": "127.0.0.1",
            "simple_behavior_web_port": 0,
            "dsd_file": "main.dsd",
            "behavior_mode": "simple_web",
            "simple_behavior_script": "",
            "odom_frame": "odom",
            "map_frame": "map",
            "base_footprint_frame": "base_footprint",
            "use_sim_time": False,
        }
    )

    wrapper = simple_behavior_web_wrapper.SimpleBehaviorWebWrapper(cast(Node, dummy_node))
    server = wrapper._http_server
    assert server is not None
    port = server.server_address[1]
    base_url = f"http://127.0.0.1:{port}"

    try:
        # 1. Health check
        req = urllib.request.Request(f"{base_url}/health")
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode())
            assert data["ok"] is True
            assert data["state"] == "idle"

        # 2. Submit valid script via POST /run
        monkeypatch.setattr(wrapper._process_manager, "_start_process_locked", lambda: None)
        run_payload = json.dumps(
            {
                "name": "test_script.py",
                "script": "def main(): pass\n",
            }
        ).encode("utf-8")
        req = urllib.request.Request(f"{base_url}/run", data=run_payload, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode())
            assert data["ok"] is True

        # 3. Submit invalid script via POST /run returns 400
        bad_run_payload = json.dumps(
            {
                "name": "bad.py",
                "script": "def invalid_syntax(:",
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            f"{base_url}/run", data=bad_run_payload, headers={"Content-Type": "application/json"}
        )
        with pytest.raises(urllib.error.HTTPError) as exc_info:
            urllib.request.urlopen(req)
        assert exc_info.value.code == 400

    finally:
        wrapper.shutdown()
