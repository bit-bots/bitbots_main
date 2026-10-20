#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard
import os
import re
import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist, Vector3
from livelybot_msg.msg import PowerSwitch
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from bitbots_msgs.action import PlayAnimation
from bitbots_msgs.msg import HeadMode, JointCommand
from bitbots_msgs.srv import MoveBall, SimulatorPush

msg = """
Bit-Bots Teleop
---------------

ESC: EMERGENCY STOP (servo power off)
SPACE: walk in place
f: full stop             F: play walkready animation
g: walk on the spot, clear speeds

Walk around:            Move head:
q    w    e         u    i    o
a    s    d         j    k    l
                    m    ,    .

q/e: turn left/right    k: zero head position
a/d: left/right         i/,: up/down
w/s: forward/back       j/l: left/right
                        u/o/m/.: combinations

Controls:
Hold keys to walk (stops on release).
SHIFT increases speed.

Head Modes:
0: Track the last known ball position
1: Look generally for all features on the field (ball, goals, corners, center point)
2: Simply look directly forward
3: Don't move the head
4: Ball Mode adapted for Penalty Kick
5: Do a pattern which only looks in front of the robot

Simulation only:
r: reset robot in simulation
R: reset ball in simulation
Move ball by 1m:
t/b: x +/-           h/n: y +/-
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
    "W": (2, 0, 0),
    "S": (-2, 0, 0),
    "A": (0, 2, 0),
    "D": (0, -2, 0),
    "Q": (0, 0, 2),
    "E": (0, 0, -2),
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
# Simulation only: move the ball by 1m along world x/y (dx, dy).
ball_move_bindings = {
    "t": (1, 0),
    "b": (-1, 0),
    "h": (0, 1),
    "n": (0, -1),
}


class TeleopKeyboard(Node):
    def __init__(self):
        # create node
        super().__init__("TeleopKeyboard")

        self.kitty_supported = False
        self._key_buffer = []
        self.pressed_keys: set[str] = set()

        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
            self.kitty_supported = self.check_kitty_support()
            if self.kitty_supported:
                sys.stdout.write("\x1b[>3u")
                sys.stdout.flush()
        else:
            self.settings = None

        self.declare_parameter("key_timeout", 0.5)
        self.declare_parameter("speed_x", 0.5)
        self.declare_parameter("speed_y", 0.2)
        self.declare_parameter("speed_turn", 0.5)

        self.key_timeout = self.get_parameter("key_timeout").get_parameter_value().double_value
        self.x_speed = self.get_parameter("speed_x").get_parameter_value().double_value
        self.y_speed = self.get_parameter("speed_y").get_parameter_value().double_value
        self.turn_speed = self.get_parameter("speed_turn").get_parameter_value().double_value

        # Walking Part
        self.pub = self.create_publisher(Twist, "cmd_vel", 1)

        self.head_yaw_pos = 0.0
        self.head_pitch_pos = 0.0

        self.x = 0.0
        self.y = 0.0
        self.a_x = -1.0
        self.th = 0.0
        self.status = 0

        self.push_force_x = 0.0
        self.push_force_y = 0.0

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
        self.move_ball = self.create_client(MoveBall, "/move_ball")
        self.simulator_push = self.create_client(SimulatorPush, "/simulator_push")

        self.frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"

        self.animation_client = ActionClient(self, PlayAnimation, "animation")
        if not self.animation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Animation action server not available after waiting 5 seconds")

        # The kick is currently disabled
        # self.kick_client = ActionClient(self, Kick, "dynamic_kick")
        # if not self.kick_client.wait_for_server(timeout_sec=0.1):
        #    self.get_logger().error("Kick action server not available after waiting 5 seconds")

        kitty_status_str = (
            "Kitty keyboard protocol: Available (key-up detection enabled)"
            if self.kitty_supported
            else "Kitty keyboard protocol: Not available (using timeout fallback)"
        )
        self.get_logger().info(kitty_status_str)
        print(msg)
        print(f"[{kitty_status_str}]\n")

    def check_kitty_support(self) -> bool:
        if self.settings is None:
            return False
        tty.setraw(sys.stdin.fileno())
        try:
            sys.stdout.write("\x1b[?u")
            sys.stdout.flush()
            rlist, _, _ = select.select([sys.stdin.fileno()], [], [], 0.2)
            if rlist:
                raw_bytes = os.read(sys.stdin.fileno(), 1024)
                while True:
                    rlist, _, _ = select.select([sys.stdin.fileno()], [], [], 0.02)
                    if not rlist:
                        break
                    raw_bytes += os.read(sys.stdin.fileno(), 1024)
                resp = raw_bytes.decode("utf-8", errors="replace")
                if bool(re.search(r"\x1b\[\?[\d;:]*u", resp)):
                    return True
            return False
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def parse_key(self, key_str: str) -> tuple[str, int]:
        """
        Parses raw key string into (char, event_type):
        event_type: 1 = press, 2 = repeat, 3 = release.
        For legacy/plain characters, event_type is 1.
        """
        if not key_str:
            return "", 0
        if key_str.startswith("\x1b[") and key_str.endswith("u") and not key_str.startswith("\x1b[?"):
            content = key_str[2:-1]
            parts = content.split(";")
            codepoint_part = parts[0]
            codepoint_sub = codepoint_part.split(":")
            codepoint = int(codepoint_sub[0]) if codepoint_sub[0] else 0
            shifted_codepoint = int(codepoint_sub[1]) if len(codepoint_sub) > 1 and codepoint_sub[1] else None

            modifiers = 1
            event_type = 1
            if len(parts) > 1:
                mod_sub = parts[1].split(":")
                if mod_sub[0]:
                    modifiers = int(mod_sub[0])
                if len(mod_sub) > 1 and mod_sub[1]:
                    event_type = int(mod_sub[1])

            is_shift = bool((modifiers - 1) & 1)
            is_ctrl = bool((modifiers - 1) & 4)

            if codepoint == 27:
                char = "\x1b"
            elif codepoint == 32:
                char = " "
            elif codepoint == 13:
                char = "\r"
            elif codepoint == 9:
                char = "\t"
            elif codepoint == 127:
                char = "\x7f"
            elif is_ctrl and (codepoint == 99 or codepoint == 67):
                char = "\x03"
            elif codepoint:
                if shifted_codepoint is not None and is_shift:
                    char = chr(shifted_codepoint)
                else:
                    base_char = chr(codepoint)
                    char = base_char.upper() if is_shift and base_char.isalpha() else base_char
            else:
                char = ""

            return char, event_type

        return key_str, 1

    def get_key(self, timeout=None):
        if self.settings is None:
            return ""
        if self._key_buffer:
            return self._key_buffer.pop(0)

        if timeout is None and not self.kitty_supported:
            timeout = self.key_timeout

        tty.setraw(sys.stdin.fileno())
        try:
            select_args = [[sys.stdin.fileno()], [], []]
            if timeout is not None:
                rlist, _, _ = select.select(*select_args, timeout)
            else:
                rlist, _, _ = select.select(*select_args)
            if not rlist:
                return ""
            raw_bytes = os.read(sys.stdin.fileno(), 1024)
            if not raw_bytes:
                return ""
            text = raw_bytes.decode("utf-8", errors="replace")
            # If text ends with an incomplete escape sequence, wait briefly for remaining bytes
            if text.startswith("\x1b") and not (
                text.endswith("u") or text.endswith("~") or (len(text) > 1 and text[-1].isalpha())
            ):
                rlist, _, _ = select.select([sys.stdin.fileno()], [], [], 0.02)
                if rlist:
                    extra_bytes = os.read(sys.stdin.fileno(), 1024)
                    text += extra_bytes.decode("utf-8", errors="replace")

            pattern = re.compile(r"(\x1b\[[0-9:;?<>=]*[a-zA-Z~]|\x1bO[a-zA-Z]|\x1b|[\s\S])")
            tokens = pattern.findall(text)
            if not tokens:
                return ""
            self._key_buffer.extend(tokens)
            return self._key_buffer.pop(0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_walkready(self):
        self.animation_client.send_goal_async(PlayAnimation.Goal(animation="walkready"))

    def _update_walk_velocities(self):
        if self.pressed_keys:
            vx = sum(move_bindings[k][0] for k in self.pressed_keys)
            vy = sum(move_bindings[k][1] for k in self.pressed_keys)
            vth = sum(move_bindings[k][2] for k in self.pressed_keys)
            self.x = round(vx * self.x_speed, 2)
            self.y = round(vy * self.y_speed, 2)
            self.th = round(vth * self.turn_speed, 2)
            self.a_x = 0.0
        else:
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0
            self.a_x = -1.0

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
                info: str = ""
                raw_key = self.get_key()
                key, event_type = self.parse_key(raw_key)

                if event_type == 3:
                    # Key release event
                    if key in move_bindings.keys():
                        if self.kitty_supported:
                            self.pressed_keys.discard(key.lower())
                            self.pressed_keys.discard(key.upper())
                            self._update_walk_velocities()
                        else:
                            self.x = 0.0
                            self.y = 0.0
                            self.th = 0.0
                            self.a_x = -1.0
                    else:
                        continue
                elif key in move_bindings.keys():
                    if self.kitty_supported:
                        self.pressed_keys.discard(key.lower())
                        self.pressed_keys.discard(key.upper())
                        self.pressed_keys.add(key)
                        self._update_walk_velocities()
                    else:
                        self.x = round(move_bindings[key][0] * self.x_speed, 2)
                        self.y = round(move_bindings[key][1] * self.y_speed, 2)
                        self.th = round(move_bindings[key][2] * self.turn_speed, 2)
                        self.a_x = 0.0
                elif key in head_bindings.keys():
                    self.head_msg.positions[0] = self.head_yaw_pos + head_bindings[key][1] * self.head_yaw_step
                    self.head_msg.positions[1] = self.head_pitch_pos + head_bindings[key][0] * self.head_pitch_step
                    self.head_pub.publish(self.head_msg)
                elif key == "k" or key == "K":
                    # put head back in init
                    self.head_msg.positions[0] = 0.0
                    self.head_msg.positions[1] = 0.0
                    self.head_pub.publish(self.head_msg)
                elif key == "0":
                    # Track the last known ball position
                    self.head_mode_msg.head_mode = HeadMode.TRACK_BALL
                    assert int(key) == HeadMode.TRACK_BALL
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
                elif key in ball_move_bindings.keys():
                    # move ball by 1m along world x/y in sim
                    try:
                        move_request = MoveBall.Request()
                        move_request.offset.x = float(ball_move_bindings[key][0])
                        move_request.offset.y = float(ball_move_bindings[key][1])
                        self.move_ball.call_async(move_request)
                    except Exception:
                        pass
                elif key == "f":
                    # complete walk stop
                    self.pressed_keys.clear()
                    self.x = 0.0
                    self.y = 0.0
                    self.a_x = -1.0
                    self.th = 0.0
                elif key == " ":
                    self.x = 0.0
                    self.y = 0.0
                    self.a_x = 0.0
                    self.th = 0.0
                    self.push_force_x = 0.0
                    self.push_force_y = 0.0
                elif key == "\x1b":  # ESC
                    self.power_switch_pub.publish(PowerSwitch(control_switch=1, power_switch=0))
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
                elif key in ("g", "G"):
                    self.pressed_keys.clear()
                    self.x = 0.0
                    self.y = 0.0
                    self.a_x = 0.0
                    self.th = 0.0
                elif key == "\x03":  # CTRL-C
                    self.pressed_keys.clear()
                    self.a_x = -1.0
                    break
                elif key == "":
                    if self.kitty_supported:
                        continue
                    # Timeout reached without any key press (legacy fallback)
                    if self.x == 0.0 and self.y == 0.0 and self.th == 0.0 and self.a_x == -1.0:
                        continue
                    self.x = 0.0
                    self.y = 0.0
                    self.th = 0.0
                    self.a_x = -1.0
                else:
                    info = f"unknown key: '{key}'"
                info = str(self.pressed_keys)

                self.head_mode_pub.publish(self.head_mode_msg)

                twist = Twist()
                twist.linear = Vector3(x=float(self.x), y=float(self.y), z=0.0)
                twist.angular = Vector3(x=float(self.a_x), y=0.0, z=float(self.th))
                self.pub.publish(twist)
                key_display = f"{key} (released)" if event_type == 3 else key
                state_str = (
                    f"x:    {self.x}     \n"
                    f"y:    {self.y}     \n"
                    f"turn: {self.th}     \n"
                    f"head mode: {self.head_mode_msg.head_mode}     \n"
                    f"push force x (+forward/-back): {self.push_force_x}     \n"
                    f"push force y (+left/-right):   {self.push_force_y}     \n"
                    f"last key: '{key_display}'    {info + ' ' * 40 if info else ' ' * 40}"
                )

                # move cursor up to overwrite previous state output
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

            if self.kitty_supported:
                sys.stdout.write("\x1b[<u")
                sys.stdout.flush()

            if self.settings is not None:
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
