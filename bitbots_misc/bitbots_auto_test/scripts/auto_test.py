#!/usr/bin/env python3
from __future__ import annotations

import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node

from bitbots_msgs.srv import SetObjectPose, SetObjectPosition


class TestCase:
    def __init__(self, handle: AutoTest):
        self.handle = handle

    def setup(self):
        pass

    def judge(self) -> float:
        return 0

    def is_over(self) -> bool:
        pass

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
        self.handle.set_robot_pose_service.call_async(request)

    def set_ball(self, x: float, y: float):
        request = SetObjectPosition.Request()
        request.position.x = x
        request.position.y = y
        self.handle.set_ball_pos_service.call_async(request)


class Line(TestCase):
    """Robot - Ball - Goal"""

    def setup(self):
        self.set_robot(0.0, 0.0)
        self.set_ball(1.5, 0.0)


test_cases = {"l": Line}

msg = f"""
BitBots Auto Test
-----------------
r : Restart the current test
Test Cases:
  {"\n  ".join(f"{k}: {t.__name__:20} {t.__doc__.strip().splitlines()[0]}" for k, t in test_cases.items())}

CTRL-C to quit





"""


class AutoTest(Node):
    def __init__(self):
        # create node
        super().__init__("AutoTest")

        self.settings = termios.tcgetattr(sys.stdin)
        self.current_test = None
        self.set_robot_pose_service = self.create_client(SetObjectPose, "set_robot_pose")
        self.set_ball_pos_service = self.create_client(SetObjectPosition, "set_ball_position")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                if key in test_cases:
                    self.current_test = test_cases[key](self)
                    self.current_test.setup()
                elif key == "r":
                    self.current_test.setup()
                elif key == "\x03":
                    break
                if self.current_test is not None:
                    state_str = (
                        f"Current test: {self.current_test.__class__.__name__}\n"
                        f"Score       : {self.current_test.judge()}\n"
                        f"Is done     : {self.current_test.is_over()}"
                    )
                else:
                    state_str = "Current test: None\nScore       : \nIs done     : \n"

                for _ in range(state_str.count("\n") + 1):
                    sys.stdout.write("\x1b[A")
                print(state_str)

        except Exception as e:
            print(e)

        finally:
            print("\n")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    rclpy.init(args=None)
    node = AutoTest()
    # necessary so that sleep in loop() is not blocking
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.loop()

    node.destroy_node()
    rclpy.shutdown()
