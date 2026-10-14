#!/usr/bin/env python3

import threading
import time
from typing import cast

import pytest
from bitbots_msgs.msg import HeadMode
from rclpy.node import Node

from bitbots_body_behavior import body_behavior
from bitbots_body_behavior import simple_behavior
from bitbots_body_behavior import simple_behavior_web_wrapper


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


class DummyClockNow:
    @staticmethod
    def to_msg():
        return None


class DummyClock:
    @staticmethod
    def now():
        return DummyClockNow()


class DummyNode:
    def __init__(self, parameters=None):
        self.logger = DummyLogger()
        self.parameters = parameters or {}

    @staticmethod
    def get_clock():
        return DummyClock()

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
    def forget_ball():
        pass


class DummyBlackboard:
    def __init__(self):
        self.world_model = DummyWorldModel()
        self.pathfinding = DummyPathfinding()
        self.kick = DummyKick()
        self.misc = DummyMisc()


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
    assert current_pose == (1.0, 2.0, 3.0)
    assert current_pose is not None
    assert current_pose.x == 1.0
    assert current_pose.y == 2.0
    assert current_pose.yaw_rad == 3.0

    ball_pose = simple_behavior.get_ball_pose()
    assert ball_pose == (4.0, 5.0)
    assert ball_pose is not None
    assert ball_pose.x == 4.0
    assert ball_pose.y == 5.0


def test_simple_behavior_requires_runtime(monkeypatch):
    monkeypatch.setattr(simple_behavior, "_runtime", None)

    with pytest.raises(RuntimeError):
        simple_behavior.get_current_pose()


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

    def fake_popen(command, start_new_session, text, env):
        del command, start_new_session, text, env
        process = FakeProcess()
        started_processes.append(process)
        return process

    monkeypatch.setattr(simple_behavior_web_wrapper.subprocess, "Popen", fake_popen)

    manager = simple_behavior_web_wrapper.SimpleBehaviorProcessManager(cast(Node, DummyNode()))
    manager._script_path = tmp_path / "simple_behavior_script.py"

    accepted, _ = manager.submit_script("def main():\n    pass\n", "first.py")
    assert accepted
    assert len(started_processes) == 1
    first_process = started_processes[0]
    assert manager.get_state() == "running"

    accepted, _ = manager.submit_script("def main():\n    pass\n", "second.py")
    assert accepted
    assert len(started_processes) == 2
    assert first_process.terminated
    assert manager.get_state() == "running"

    manager.shutdown()
    assert started_processes[1].terminated


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
    wrapper._node = cast(Node, DummyNode(parameters={"simple_behavior_web_host": "127.0.0.1", "simple_behavior_web_port": 8765}))
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
