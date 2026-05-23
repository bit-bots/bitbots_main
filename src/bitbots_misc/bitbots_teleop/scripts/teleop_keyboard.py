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
from geometry_msgs.msg import Twist, Vector3
from livelybot_msg.msg import PowerSwitch
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from bitbots_msgs.action import PlayAnimation
from bitbots_msgs.msg import HeadMode, JointCommand
from bitbots_msgs.srv import SimulatorPush

msg = """
Bit-Bots Teleop
---------------

SPACE: EMERGENCY STOP (servo power off)
f: full stop             F: play walkready animation

Walk around:            Move head:
q    w    e         u    i    o
a    s    d         j    k    l
                    m    ,    .

q/e: turn left/right    k: zero head position
a/d: left/right         i/,: up/down
w/s: forward/back       j/l: left/right
                        u/o/m/.: combinations

Controls increase / decrease with multiple presses.
SHIFT increases with factor 10

Head Modes:
(Currently not implemented: 0: Track the last known ball position)
1: Look generally for all features on the field (ball, goals, corners, center point)
2: Simply look directly forward
3: Don't move the head
4: Ball Mode adapted for Penalty Kick
5: Do a pattern which only looks in front of the robot

Simulation only:
r: reset robot in simulation
R: reset ball in simulation
p: execute Push
P: reset Power to 0
ü/ä: increase/decrease power forward (x axis)
+/#: increase/decrease power left (y axis)
SHIFT increases/decreases with factor 10

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

        self.head_yaw_pos = 0
        self.head_pitch_pos = 0

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
        self.head_mode_pub = self.create_publisher(HeadMode, "head_mode", 1)
        if self.get_parameter("use_sim_time").get_parameter_value().bool_value:
            self.head_pub = self.create_publisher(JointCommand, "head_motor_goals", 1)
        else:
            self.head_pub = self.create_publisher(JointCommand, "joint_command", 1)

        self.head_msg = JointCommand()
        self.head_msg.max_torques = [-1.0] * 2
        self.head_msg.velocities = [5.0] * 2
        self.head_msg.accelerations = [40.0] * 2
        self.head_msg.joint_names = ["head_yaw_joint", "head_pitch_joint"]
        self.head_msg.positions = [0.0] * 2

        self.head_mode_msg = HeadMode(head_mode=HeadMode.DONT_MOVE)

        self.head_yaw_step = 0.05
        self.head_pitch_step = 0.05

        self.power_switch_pub = self.create_publisher(PowerSwitch, "/power_switch_control", 10)

        self.reset_robot = self.create_client(Empty, "/reset_pose")
        self.reset_ball = self.create_client(Empty, "/reset_ball")
        self.simulator_push = self.create_client(SimulatorPush, "/simulator_push")

        self.frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"

        self.animation_client = ActionClient(self, PlayAnimation, "animation")
        if not self.animation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Animation action server not available after waiting 5 seconds")

        # The kick is currently disabled
        # self.kick_client = ActionClient(self, Kick, "dynamic_kick")
        # if not self.kick_client.wait_for_server(timeout_sec=0.1):
        #    self.get_logger().error("Kick action server not available after waiting 5 seconds")

        print(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def get_walkready(self):
        self.animation_client.send_goal_async(PlayAnimation.Goal(animation="walkready"))

    # def generate_kick_goal(self, x, y, direction):
    #     kick_goal = Kick.Goal()
    #     kick_goal.header.stamp = self.get_clock().now().to_msg()
    #     kick_goal.header.frame_id = self.frame_prefix + "base_footprint"
    #     kick_goal.ball_position = Point(x=float(x), y=float(y), z=0.0)
    #     kick_goal.kick_direction = quat_from_yaw(direction)
    #     kick_goal.kick_speed = 1.0
    #     return kick_goal

    def joint_state_cb(self, msg):
        if "head_yaw_joint" in msg.name and "head_pitch_joint" in msg.name:
            self.head_yaw_pos = msg.position[msg.name.index("head_yaw_joint")]
            self.head_pitch_pos = msg.position[msg.name.index("head_pitch_joint")]

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
                    self.head_msg.positions[0] = self.head_yaw_pos + head_bindings[key][1] * self.head_yaw_step
                    self.head_msg.positions[1] = self.head_pitch_pos + head_bindings[key][0] * self.head_pitch_step
                    self.head_pub.publish(self.head_msg)
                elif key == "k" or key == "K":
                    # put head back in init
                    self.head_msg.positions[0] = 0
                    self.head_msg.positions[1] = 0
                    self.head_pub.publish(self.head_msg)
                elif key == "0":
                    # Track the last known ball position
                    # self.head_mode_msg.head_mode = HeadMode.TRACK_BALL
                    # assert int(key) == HeadMode.TRACK_BALL
                    print("ERROR: CURRENTLY NOT IMPLEMENTED")
                elif key == "1":
                    # Look generally for all features on the field (ball, goals, corners, center point)
                    self.head_mode_msg.head_mode = HeadMode.SEARCH_FIELD_FEATURES
                    assert int(key) == HeadMode.SEARCH_FIELD_FEATURES
                elif key == "2":
                    # Simply look directly forward
                    self.head_mode_msg.head_mode = HeadMode.LOOK_FORWARD
                    assert int(key) == HeadMode.LOOK_FORWARD
                elif key == "3":
                    # Don't move the head
                    self.head_mode_msg.head_mode = HeadMode.DONT_MOVE
                    assert int(key) == HeadMode.DONT_MOVE
                elif key == "4":
                    # Ball Mode adapted for Penalty Kick
                    self.head_mode_msg.head_mode = HeadMode.SEARCH_BALL_PENALTY
                    assert int(key) == HeadMode.SEARCH_BALL_PENALTY
                elif key == "5":
                    # Do a pattern which only looks in front of the robot
                    self.head_mode_msg.head_mode = HeadMode.SEARCH_FRONT
                    assert int(key) == HeadMode.SEARCH_FRONT
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
                elif key == " ":
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.a_x = 0
                    self.th = 0
                    self.push_force_x = 0.0
                    self.push_force_y = 0.0
                    self.power_switch_pub.publish(PowerSwitch(control_switch=0, power_switch=0))
                elif key == "p":
                    # push robot in simulation
                    push_request = SimulatorPush.Request()
                    push_request.force.x = self.push_force_x
                    push_request.force.y = self.push_force_y
                    push_request.relative = True
                    self.simulator_push.call_async(push_request)
                elif key == "P":
                    self.push_force_x = 0.0
                    self.push_force_y = 0.0
                elif key == "ü":
                    self.push_force_x += 1
                elif key == "Ü":
                    self.push_force_x += 10
                elif key == "ä":
                    self.push_force_x -= 1
                elif key == "Ä":
                    self.push_force_x -= 10
                elif key == "+":
                    self.push_force_y += 1
                elif key == "*":
                    self.push_force_y += 10
                elif key == "#":
                    self.push_force_y -= 1
                elif key == "'":
                    self.push_force_y -= 10
                else:
                    if key == "\x03":  # CTRL-C
                        self.a_x = -1
                        break

                self.head_mode_pub.publish(self.head_mode_msg)

                twist = Twist()
                twist.linear = Vector3(x=float(self.x), y=float(self.y), z=0.0)
                twist.angular = Vector3(x=float(self.a_x), y=0.0, z=float(self.th))
                self.pub.publish(twist)
                state_str = (
                    f"x:    {self.x}     \n"
                    f"y:    {self.y}     \n"
                    f"turn: {self.th}     \n"
                    f"head mode: {self.head_mode_msg.head_mode}     \n"
                    f"push force x (+forward/-back): {self.push_force_x}     \n"
                    f"push force y (+left/-right):   {self.push_force_y}     "
                )

                for _ in range(state_str.count("\n") + 1):
                    sys.stdout.write("\x1b[A")
                print(state_str)

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
