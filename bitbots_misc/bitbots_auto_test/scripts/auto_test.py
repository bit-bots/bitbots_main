#!/usr/bin/env python3
from __future__ import annotations

import select
import sys
import termios
import threading
import tty

import rclpy
import rclpy.executors
from bitbots_auto_test.monitoring_node import Monitoring
from game_controller_hl_interfaces.msg import GameState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
from rclpy.clock import Duration
from rclpy.constants import S_TO_NS
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile

from bitbots_msgs.srv import SetObjectPose, SetObjectPosition


class TestCase:
    def __init__(self, handle: AutoTest, test_id: int, test_repeat: int):
        self.handle = handle
        self.test_id = test_id
        self.test_repeat = test_repeat
        self.start_time = 0
        self.end_time = 0
        self.is_done = False
        self.final_score = None

    def setup(self):
        pass

    def start_recording(self):
        self.start_time = self.handle.get_clock().now()

    def judge(self) -> float:
        pass

    def is_over(self) -> bool:
        pass

    def update(self):
        if self.is_done:
            return
        if self.is_over():
            self.end_time = self.handle.get_clock().now()
            self.final_score = self.judge()
            self.is_done = True

    def score(self):
        if self.is_done:
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

    def set_ball(self, x: float, y: float):
        request = SetObjectPosition.Request()
        request.position.x = x
        request.position.y = y
        self.handle.set_ball_pos_service.call(request)

    def is_ball_in_goal(self) -> bool:
        x = self.handle.ball_pose.position.x
        y = self.handle.ball_pose.position.y
        if x <= 4.6:
            return False
        if -1.3 <= y <= 1.3:
            return True
        return False


class MakeGoal(TestCase):
    """Robot - Ball - Goal"""

    def setup(self):
        self.set_robot(0.0, 0.0)
        self.set_ball(0.75, 0.0)
        super().setup()

    def judge(self):
        return (self.handle.get_clock().now() - self.start_time).nanoseconds / S_TO_NS

    def is_over(self):
        return self.is_ball_in_goal()


test_cases = {"g": MakeGoal}

msg = f"""
BitBots Auto Test
-----------------
r : Restart the current test
Test Cases:
  {"\n  ".join(f"{k}: {t.__name__:20} {t.__doc__.strip().splitlines()[0]}" for k, t in test_cases.items())}

CTRL-C to quit





"""
TEAM_ID = 6


class AutoTest(Node):
    def __init__(self, monitoring_node: Monitoring):
        # create node
        super().__init__("AutoTest")

        use_sim_time_param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        self.set_parameters([use_sim_time_param])

        self.monitoring_node = monitoring_node

        self.settings = termios.tcgetattr(sys.stdin)
        self.test_id = 0
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

    def model_states_callback(self, model_state_msg: ModelStates):
        for i, name in enumerate(model_state_msg.name):
            if name == "amy":
                self.robot_pose = model_state_msg.pose[i]
            elif name == "ball":
                self.ball_pose = model_state_msg.pose[i]
                self.ball_twist = model_state_msg.twist[i]

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if not rlist:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return None
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def next_test(self):
        if self.current_test is not None:
            self.test_id += 1
            self.current_test = self.current_test.__class__(self, self.test_id, self.current_test.test_repeat + 1)

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
            self.current_test.__class__.__name__,
            str(self.current_test.score()),
        )
        self.monitoring_node.write_reduced_pose(
            "test_end_robot_pose", self.monitoring_node.last_robot_pose, self.monitoring_node.last_model_states_time
        )
        self.monitoring_node.write_reduced_pose(
            "test_end_ball_pose", self.monitoring_node.last_ball_pose, self.monitoring_node.last_model_states_time
        )

    def loop(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                if key in test_cases:
                    self.current_test = test_cases[key](self, self.test_id, 1)
                    self.setup_sequence()
                elif key == "r":
                    if self.current_test is not None:
                        self.setup_sequence()
                elif key == "\x03":
                    break
                if self.current_test is not None:
                    self.current_test.update()
                    state_str = (
                        f"Current test: {self.current_test.__class__.__name__}\n"
                        f"Score       : {self.current_test.score()}\n"
                        f"Is done     : {self.current_test.is_done}\n"
                    )
                else:
                    state_str = "Current test: None\nScore       : \nIs done     : \n"

                for _ in range(state_str.count("\n") + 1):
                    sys.stdout.write("\x1b[A")
                print(state_str)
                if self.current_test is not None and self.current_test.is_done:
                    self.finished()
                    self.next_test()
                    self.setup_sequence()

        except Exception as e:
            print(e)

        finally:
            print("\n")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    rclpy.init(args=None)
    monitoring = Monitoring()
    auto_test = AutoTest(monitoring)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(monitoring)
    executor.add_node(auto_test)

    # necessary so that sleep in loop() is not blocking
    thread = threading.Thread(target=executor.spin, args=(), daemon=True)
    thread.start()
    auto_test.loop()

    auto_test.destroy_node()
    monitoring.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
