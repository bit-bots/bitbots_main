#!/usr/bin/env python3

import pytest

from bitbots_body_behavior import simple_behavior


class DummyLogger:
    def warn(self, _: str):
        pass


class DummyClockNow:
    @staticmethod
    def to_msg():
        return None


class DummyClock:
    @staticmethod
    def now():
        return DummyClockNow()


class DummyNode:
    @staticmethod
    def get_clock():
        return DummyClock()

    @staticmethod
    def get_logger():
        return DummyLogger()


class DummyPathfinding:
    def __init__(self):
        self.last_goal = None

    def publish(self, goal):
        self.last_goal = goal


class DummyKick:
    def __init__(self):
        self.last_kick = None

    def start_rl_kick(self, direction_deg, strength):
        self.last_kick = (direction_deg, strength)
        return True


class DummyWorldModel:
    map_frame = "map"

    @staticmethod
    def get_current_position():
        return 1.0, 2.0, 3.0

    @staticmethod
    def get_ball_position_xy():
        return 4.0, 5.0


class DummyBlackboard:
    def __init__(self):
        self.world_model = DummyWorldModel()
        self.pathfinding = DummyPathfinding()
        self.kick = DummyKick()


def test_simple_behavior_api_forwards_to_runtime(monkeypatch):
    runtime = simple_behavior.SimpleBehaviorRuntime(DummyNode(), DummyBlackboard())
    monkeypatch.setattr(simple_behavior, "_runtime", runtime)

    simple_behavior.set_goal_pose(0.1, 0.2, 15.0)
    assert runtime._blackboard.pathfinding.last_goal is not None
    assert runtime._blackboard.pathfinding.last_goal.pose.position.x == 0.1
    assert runtime._blackboard.pathfinding.last_goal.pose.position.y == 0.2

    assert simple_behavior.kick_ball(45.0, 2.5)
    assert runtime._blackboard.kick.last_kick == (45.0, 2.5)

    assert simple_behavior.get_current_pose() == (1.0, 2.0, 3.0)
    assert simple_behavior.get_ball_pose() == (4.0, 5.0)


def test_simple_behavior_requires_runtime(monkeypatch):
    monkeypatch.setattr(simple_behavior, "_runtime", None)

    with pytest.raises(RuntimeError):
        simple_behavior.get_current_pose()
