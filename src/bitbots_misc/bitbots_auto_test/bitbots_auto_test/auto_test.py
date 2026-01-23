#!/usr/bin/env python3
from __future__ import annotations

import threading
from dataclasses import dataclass

import rclpy
import rclpy.executors
from bitbots_localization.srv import ResetFilter
from game_controller_hl_interfaces.msg import GameState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
from rclpy.clock import Duration
from rclpy.constants import S_TO_NS
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile

from bitbots_auto_test.monitoring_node import Monitoring
from bitbots_msgs.srv import SetObjectPose, SetObjectPosition


class TestResult:
    pass


@dataclass
class TestSuccess:
    pass


@dataclass
class TestFailure(TestResult):
    reason: str


class TestCase:
    FIELD_BOUNDS = ((-5.5, -4), (5.5, 4))
    PLAY_BOUNDS = ((-4.5, -3), (4.5, 3))
    GOAL_BOUNDS = ((4.6, -1.3), (5.5, 1.3))
    TIMEOUT = Duration(seconds=5 * 60)

    def __init__(self, handle: AutoTest, test_id: int, test_repeat: int):
        self.handle = handle
        self.test_id = test_id
        self.test_repeat = test_repeat
        self.start_time = 0
        self.end_time = 0
        self.result: None | TestResult = None
        self.final_score = None

    def setup(self):
        pass

    def start_recording(self):
        self.start_time = self.handle.get_clock().now()

    def judge(self) -> float:
        pass

    def is_over(self) -> TestResult | None:
        if not self.is_pose_in(self.handle.robot_pose, self.FIELD_BOUNDS):
            return TestFailure("robot_out_of_bounds")
        if not self.is_pose_in(self.handle.ball_pose, self.FIELD_BOUNDS):
            return TestFailure("ball_out_of_bounds")
        if self.start_time + self.TIMEOUT < self.handle.get_clock().now():
            return TestFailure("timeout")

    def update(self):
        if self.result is not None:
            return
        if res := self.is_over():
            self.end_time = self.handle.get_clock().now()
            self.final_score = self.judge()
            self.result = res

    def score(self):
        if self.result is not None:
            return self.final_score
        else:
            return self.judge()

    def set_robot(self, x: float, y: float):
        request = SetObjectPose.Request()
        request.object_name = "amy"
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = 0.42
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0
        request.pose.orientation.w = 0.0
        self.handle.set_robot_pose_service.call(request)
        if not self.handle.get_parameter("fake_localization").value:
            rf = ResetFilter.Request()
            rf.init_mode = ResetFilter.Request.POSE
            rf.x = x
            rf.y = y
            rf.angle = 0.0
            self.handle.reset_localization.call(rf)

    def set_ball(self, x: float, y: float):
        request = SetObjectPosition.Request()
        request.position.x = x
        request.position.y = y
        self.handle.set_ball_pos_service.call(request)

    def is_pose_in(self, pose, rect) -> bool:
        x = pose.position.x
        y = pose.position.y
        (ax, ay), (bx, by) = rect
        return ax <= x <= bx and ay <= y <= by

    def is_ball_in_goal(self) -> bool:
        return self.is_pose_in(self.handle.ball_pose, self.GOAL_BOUNDS)


class MakeGoalStraight(TestCase):
    """Robot - Ball - Goal"""

    def setup(self):
        self.set_robot(0.0, 0.0)
        self.set_ball(0.75, 0.0)
        super().setup()

    def judge(self):
        return (self.handle.get_clock().now() - self.start_time).nanoseconds / S_TO_NS

    def is_over(self):
        if self.is_ball_in_goal():
            return TestSuccess()
        return super().is_over()


class MakeGoalDiagonalLeft(TestCase):
    def setup(self):
        self.set_robot(0.0, 0.0)
        self.set_ball(0.75, 2.6)
        super().setup()

    def judge(self):
        return (self.handle.get_clock().now() - self.start_time).nanoseconds / S_TO_NS

    def is_over(self):
        if self.is_ball_in_goal():
            return TestSuccess()
        return super().is_over()


class MakeGoalDiagonalRight(TestCase):
    def setup(self):
        self.set_robot(0.0, 0.0)
        self.set_ball(0.75, -2.6)
        super().setup()

    def judge(self):
        return (self.handle.get_clock().now() - self.start_time).nanoseconds / S_TO_NS

    def is_over(self):
        if self.is_ball_in_goal():
            return TestSuccess()
        return super().is_over()


class MakeGoalClose(TestCase):
    """Robot - Ball - Goal"""

    def setup(self):
        self.set_robot(4.6 - 1.5, 0.0)
        self.set_ball(4.6 - 0.75, 0.0)
        super().setup()

    def judge(self):
        return (self.handle.get_clock().now() - self.start_time).nanoseconds / S_TO_NS

    def is_over(self):
        if self.is_ball_in_goal():
            return TestSuccess()
        return super().is_over()


class MakeGoalBehind(TestCase):
    """Ball - Robot - Goal"""

    def setup(self):
        self.set_robot(4.6 - 1.5, 0.0)
        self.set_ball(4.6 - 3, 0.0)
        super().setup()

    def judge(self):
        return (self.handle.get_clock().now() - self.start_time).nanoseconds / S_TO_NS

    def is_over(self):
        if self.is_ball_in_goal():
            return TestSuccess()
        return super().is_over()


TEAM_ID = 6
TEST_CASES = {c.__name__: c for c in TestCase.__subclasses__()}


class AutoTest(Node):
    def __init__(self, monitoring_node: Monitoring):
        # create node
        super().__init__("auto_test")

        self.declare_parameter("fake_localization", True)
        self.declare_parameter("loop", False)
        self.declare_parameter("repeats", 1)
        self.declare_parameter("test_cases", ["MakeGoalStraight"])
        use_sim_time_param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        self.set_parameters([use_sim_time_param])

        self.monitoring_node = monitoring_node

        self.test_id = 0
        self.test_type_index = 0
        self.current_test = None

        self.set_robot_pose_service = self.create_client(SetObjectPose, "set_robot_pose")
        self.set_ball_pos_service = self.create_client(SetObjectPosition, "set_ball_position")
        self.create_subscription(ModelStates, "/model_states", qos_profile=10, callback=self.model_states_callback)
        self.ball_pose = Pose()
        self.ball_twist = Twist()
        self.robot_pose = Pose()

        self.gamestate_publisher = self.create_publisher(
            GameState,
            "gamestate",
            QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1),
        )
        if not self.get_parameter("fake_localization").value:
            self.reset_localization = self.create_client(ResetFilter, "reset_localization")

    def model_states_callback(self, model_state_msg: ModelStates):
        for i, name in enumerate(model_state_msg.name):
            if name == "amy":
                self.robot_pose = model_state_msg.pose[i]
            elif name == "ball":
                self.ball_pose = model_state_msg.pose[i]
                self.ball_twist = model_state_msg.twist[i]

    def next_test(self):
        test_repeats = self.get_parameter("repeats").value
        test_cases = self.get_parameter("test_cases").value
        do_loop = self.get_parameter("loop").value
        if self.current_test is not None:
            self.test_id += 1
            if self.current_test.test_repeat >= test_repeats:
                self.test_type_index += 1
                if self.test_type_index >= len(test_cases):
                    if not do_loop:
                        self.current_test = None
                        return
                    else:
                        self.test_type_index = 0
                rep = 1
            else:
                rep = self.current_test.test_repeat + 1
        else:
            rep = 1
            self.test_id += 1
        test_cls = TEST_CASES[test_cases[self.test_type_index]]
        self.current_test = test_cls(self, self.test_id, rep)

    def setup_sequence(self):
        gs_msg = GameState()
        gs_msg.header.stamp = self.get_clock().now().to_msg()
        gs_msg.secondary_state_team = TEAM_ID
        gs_msg.game_state = 2  # 2=SET, force robot to stand still
        self.gamestate_publisher.publish(gs_msg)

        self.get_clock().sleep_for(Duration(seconds=2))  # Let robot react & simulation settle
        self.current_test.setup()  # Teleport robot

        self.get_clock().sleep_for(Duration(seconds=2))  # Let simulation settle

        gs_msg.header.stamp = self.get_clock().now().to_msg()
        gs_msg.game_state = 3  # 3=PLAYING, make robot play ball
        self.gamestate_publisher.publish(gs_msg)
        self.current_test.start_recording()
        self.monitoring_node.write_event(
            "test_start",
            str(self.current_test.test_id),
            self.current_test.__class__.__name__,
            str(self.current_test.test_repeat),
        )
        self.monitoring_node.write_reduced_pose(
            "test_start_robot_pose", self.monitoring_node.last_robot_pose, self.monitoring_node.last_model_states_time
        )
        self.monitoring_node.write_reduced_pose(
            "test_start_ball_pose", self.monitoring_node.last_ball_pose, self.monitoring_node.last_model_states_time
        )

    def finished(self):
        self.monitoring_node.write_event(
            "test_end",
            str(self.current_test.test_id),
            ("success" if isinstance(self.current_test.result, TestSuccess) else self.current_test.result.reason),
            str(self.current_test.score()),
        )
        self.monitoring_node.write_reduced_pose(
            "test_end_robot_pose", self.monitoring_node.last_robot_pose, self.monitoring_node.last_model_states_time
        )
        self.monitoring_node.write_reduced_pose(
            "test_end_ball_pose", self.monitoring_node.last_ball_pose, self.monitoring_node.last_model_states_time
        )

    def loop(self):
        self.next_test()
        self.setup_sequence()
        last_print = self.get_clock().now()
        while True:
            if self.current_test is not None:
                self.current_test.update()
            if self.get_clock().now() - last_print > Duration(seconds=2):
                last_print = self.get_clock().now()
            if self.current_test is not None and self.current_test.result is not None:
                self.finished()
                self.next_test()
                if self.current_test is None:
                    print("Done with all tests")
                    break
                self.setup_sequence()


def main():
    rclpy.init(args=None)
    monitoring = Monitoring()
    auto_test = AutoTest(monitoring)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(monitoring)
    executor.add_node(auto_test)

    thread = threading.Thread(target=executor.spin, args=(), daemon=True)
    thread.start()
    auto_test.loop()

    auto_test.destroy_node()
    monitoring.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
