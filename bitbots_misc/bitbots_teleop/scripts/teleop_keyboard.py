#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from bitbots_utils.transforms import quat_from_yaw
from geometry_msgs.msg import Point, Twist, Vector3
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from bitbots_msgs.action import Dynup, Kick
from bitbots_msgs.msg import JointCommand

msg = """
BitBots Teleop
--------------
Walk around:            Move head:
    q    w    e         u    i    o
    a    s    d         j    k    l
                        m    ,    .

q/e: turn left/right    k: zero head position
a/d: left/rigth         i/,: up/down
w/s: forward/back       j/l: left/right
                        u/o/m/.: combinations

Controls increase / decrease with multiple presses.
SHIFT increases with factor 10

y: kick left forward   Y: walk kick left forward
c: kick right forward  C: walk kick right forward
<: side kick left 1    >: side kick left 2
v: side kick right 1   V: side kick right 2
x: kick center forward X: kick center backward
b: kick left backward  n: kick right backward
B: kick left outward   N: kick right outward

f: full stop           F: play walkready animation
r: reset robot in simulation
R: reset ball in simulation

CTRL-C to quit




"""

move_bindings = {
    "w": (1, 0, 0),
    "s": (-1, 0, 0),
    "a": (0, 1, 0),
    "d": (0, -1, 0),
    "q": (0, 0, 1),
    "e": (0, 0, -1),
    "W": (10, 0, 0),
    "S": (-10, 0, 0),
    "A": (0, 10, 0),
    "D": (0, -10, 0),
    "Q": (0, 0, 10),
    "E": (0, 0, -10),
}
head_bindings = {
    "u": (1, 1),
    "i": (1, 0),
    "o": (1, -1),
    "j": (0, 1),
    "l": (0, -1),
    "m": (-1, 1),
    ",": (-1, 0),
    ".": (-1, -1),
    "U": (10, 10),
    "I": (10, 0),
    "O": (10, -10),
    "J": (0, 10),
    "L": (0, -10),
    "M": (-10, 10),
    ";": (-10, 0),
    ":": (-10, -10),
}


class TeleopKeyboard(Node):
    def __init__(self):
        # create node
        super().__init__("TeleopKeyboard")

        self.settings = termios.tcgetattr(sys.stdin)

        # Walking Part
        self.pub = self.create_publisher(Twist, "cmd_vel", 1)

        self.head_pan_pos = 0
        self.head_tilt_pos = 0

        self.x_speed_step = 0.01
        self.y_speed_step = 0.01
        self.turn_speed_step = 0.01

        self.x = 0
        self.y = 0
        self.a_x = -1
        self.th = 0
        self.status = 0

        # Head Part
        self.create_subscription(JointState, "joint_states", self.joint_state_cb, 1)
        if self.get_parameter("use_sim_time").get_parameter_value().bool_value:
            self.head_pub = self.create_publisher(JointCommand, "head_motor_goals", 1)
        else:
            self.head_pub = self.create_publisher(JointCommand, "DynamixelController/command", 1)
        self.head_msg = JointCommand()
        self.head_msg.max_currents = [-1.0] * 2
        self.head_msg.velocities = [5.0] * 2
        self.head_msg.accelerations = [40.0] * 2
        self.head_msg.joint_names = ["HeadPan", "HeadTilt"]
        self.head_msg.positions = [0.0] * 2

        self.head_pan_step = 0.05
        self.head_tilt_step = 0.05

        self.walk_kick_pub = self.create_publisher(Bool, "kick", 1)

        self.reset_robot = self.create_client(Empty, "/reset_pose")
        self.reset_ball = self.create_client(Empty, "/reset_ball")

        print(msg)

        self.frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"

        self.dynup_client = ActionClient(self, Dynup, "dynup")
        if not self.dynup_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Dynup action server not available after waiting 5 seconds")

        self.kick_client = ActionClient(self, Kick, "dynamic_kick")
        if not self.kick_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Kick action server not available after waiting 5 seconds")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def get_walkready(self):
        goal = Dynup.Goal()
        goal.direction = "walkready"
        result: Dynup.Result = self.dynup_client.send_goal(goal).result
        if not result.successful:
            self.get_logger().error("Could not execute walkready animation")
        return result.successful

    def generate_kick_goal(self, x, y, direction):
        kick_goal = Kick.Goal()
        kick_goal.header.stamp = self.get_clock().now().to_msg()
        kick_goal.header.frame_id = self.frame_prefix + "base_footprint"
        kick_goal.ball_position = Point(x=float(x), y=float(y), z=0.0)
        kick_goal.kick_direction = quat_from_yaw(direction)
        kick_goal.kick_speed = 1.0
        return kick_goal

    def joint_state_cb(self, msg):
        if "HeadPan" in msg.name and "HeadTilt" in msg.name:
            self.head_pan_pos = msg.position[msg.name.index("HeadPan")]
            self.head_tilt_pos = msg.position[msg.name.index("HeadTilt")]

    def loop(self):
        try:
            while True:
                key = self.get_key()
                if key in move_bindings.keys():
                    self.x += move_bindings[key][0] * self.x_speed_step

                    self.x = round(self.x, 2)
                    self.y += move_bindings[key][1] * self.y_speed_step
                    self.y = round(self.y, 2)
                    self.th += move_bindings[key][2] * self.turn_speed_step
                    self.th = round(self.th, 2)
                    self.a_x = 0
                elif key in head_bindings.keys():
                    self.head_msg.positions[0] = self.head_pan_pos + head_bindings[key][1] * self.head_pan_step
                    self.head_msg.positions[1] = self.head_tilt_pos + head_bindings[key][0] * self.head_tilt_step
                    self.head_pub.publish(self.head_msg)
                elif key == "k" or key == "K":
                    # put head back in init
                    self.head_msg.positions[0] = 0
                    self.head_msg.positions[1] = 0
                    self.head_pub.publish(self.head_msg)
                elif key == "y":
                    # kick left forward
                    self.kick_client.send_goal_async(self.generate_kick_goal(0.2, 0.1, 0))
                elif key == "<":
                    # kick left side ball left
                    self.kick_client.send_goal_async(self.generate_kick_goal(0.2, 0.1, -1.57))
                elif key == ">":
                    # kick left side ball center
                    self.kick_client.send_goal_async(self.generate_kick_goal(0.2, 0, -1.57))
                elif key == "c":
                    # kick right forward
                    self.kick_client.send_goal_async(self.generate_kick_goal(0.2, -0.1, 0))
                elif key == "v":
                    # kick right side ball right
                    self.kick_client.send_goal_async(self.generate_kick_goal(0.2, -0.1, 1.57))
                elif key == "V":
                    # kick right side ball center
                    self.kick_client.send_goal_async(self.generate_kick_goal(0.2, 0, 1.57))
                elif key == "x":
                    # kick center forward
                    self.kick_client.send_goal_async(self.generate_kick_goal(0.2, 0, 0))
                elif key == "X":
                    # kick center backwards
                    self.kick_client.send_goal_async(self.generate_kick_goal(-0.2, 0, 0))
                elif key == "b":
                    # kick left backwards
                    self.kick_client.send_goal_async(self.generate_kick_goal(-0.2, 0.1, 0))
                elif key == "n":
                    # kick right backwards
                    self.kick_client.send_goal_async(self.generate_kick_goal(-0.2, -0.1, 0))
                elif key == "B":
                    # kick left backwards
                    self.kick_client.send_goal_async(self.generate_kick_goal(0, 0.14, -1.57))
                elif key == "N":
                    # kick right backwards
                    self.kick_client.send_goal_async(self.generate_kick_goal(0, -0.14, 1.57))
                elif key == "Y":
                    # kick left walk
                    self.walk_kick_pub.publish(Bool(data=False))
                elif key == "C":
                    # kick right walk
                    self.walk_kick_pub.publish(Bool(data=True))
                elif key == "F":
                    # play walkready animation
                    self.get_walkready()
                elif key == "r":
                    # reset robot in sim
                    try:
                        self.reset_robot.call_async(Empty.Request())
                    except Exception:
                        pass
                elif key == "R":
                    # reset ball in sim
                    try:
                        self.reset_ball.call_async(Empty.Request())
                    except Exception:
                        pass
                elif key == "f":
                    # complete walk stop
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.a_x = -1
                    self.th = 0
                else:
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.a_x = 0
                    self.th = 0
                    if key == "\x03":
                        self.a_x = -1
                        break

                twist = Twist()
                twist.linear = Vector3(x=float(self.x), y=float(self.y), z=0.0)
                twist.angular = Vector3(x=float(self.a_x), y=0.0, z=float(self.th))
                self.pub.publish(twist)
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                print(f"x:    {self.x}          \ny:    {self.y}          \nturn: {self.th}          \n\n")

        except Exception as e:
            print(e)

        finally:
            print("\n")
            twist = Twist()
            twist.angular.x = -1.0
            self.pub.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    rclpy.init(args=None)
    node = TeleopKeyboard()
    # necessary so that sleep in loop() is not blocking
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.loop()

    node.destroy_node()
    rclpy.shutdown()
