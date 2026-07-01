#!/usr/bin/env python3

# Keyboard teleop for testing the rl_kick action server.
# Sends a Kick action goal with configurable direction, strength and timeout,
# then optionally sends a walk command and a full-stop command after
# configurable delays.

import math
import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from bitbots_msgs.action import Kick

HELP = """
RL Kick Teleop
--------------
Direction — two modes, last used wins:
  w / s   x coord  +/-  (xy mode,    +x = forward)
  a / d   y coord  +/-  (xy mode,    +y = left)
  q / e   angle    +/-  (angle mode, 5° steps; 0° = forward, 90° = left)

Other kick parameters:
  r / f   strength  +/-
  t / g   timeout   +/-  (seconds; negative = node default)

Post-kick sequence (runs after the action returns):
  z / h   walk delay  +/-  (seconds before walk command is sent)
  u / j   stop delay  +/-  (seconds between walk command and full stop)

SPACE / ENTER   send kick
CTRL-C          quit (sends full stop)
"""

_STEP_XY = 0.05
_STEP_ANGLE_DEG = 5.0
_STEP_STRENGTH = 0.1
_STEP_TIMEOUT = 0.1
_STEP_DELAY = 0.1


class KickTeleop(Node):
    def __init__(self):
        super().__init__("kick_teleop")
        self._settings = termios.tcgetattr(sys.stdin)

        self._cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self._kick_client = ActionClient(self, Kick, "rl_kick")

        # Direction — either xy or angle mode depending on what was edited last
        self._mode = "angle"   # "xy" or "angle"
        self._angle_deg = 0.0  # degrees; 0 = forward, 90 = left
        self._x = 1.0          # always kept in sync with angle in angle mode
        self._y = 0.0

        self._strength = 1.0
        self._kick_timeout = 1.5  # <0 means use the node's configured default

        # Post-kick delays
        self._walk_delay = 0.1   # seconds after action returns before walk cmd
        self._stop_delay = 2.5   # seconds after walk cmd before full stop

        self._kick_in_flight = False
        self._lock = threading.Lock()

        if not self._kick_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning("rl_kick action server not available after 5 s")

    # ------------------------------------------------------------------
    # key input
    # ------------------------------------------------------------------

    def _get_key(self) -> str:
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        return key

    # ------------------------------------------------------------------
    # direction helpers
    # ------------------------------------------------------------------

    def _apply_angle(self):
        """Recompute x/y from _angle_deg (unit vector)."""
        rad = math.radians(self._angle_deg)
        self._x = round(math.cos(rad), 4)
        self._y = round(math.sin(rad), 4)

    def _sync_angle_from_xy(self):
        """Recompute _angle_deg from current x/y (informational only)."""
        self._angle_deg = round(math.degrees(math.atan2(self._y, self._x)), 1)

    # ------------------------------------------------------------------
    # kick
    # ------------------------------------------------------------------

    def _trigger_kick(self):
        with self._lock:
            if self._kick_in_flight:
                return "kick already in flight"
            self._kick_in_flight = True

        goal = Kick.Goal()
        goal.x = float(self._x)
        goal.y = float(self._y)
        goal.strength = float(self._strength)
        goal.timeout = float(self._kick_timeout)

        #self.get_logger().info(
        #    f"Kick goal: x={goal.x:.3f}  y={goal.y:.3f}  "
        #    f"strength={goal.strength:.2f}  timeout={goal.timeout:.1f}s"
        #)
        self._kick_client.send_goal_async(goal)

        # Start the post-kick timer immediately using the same timeout sent to
        # the action, so the sequence fires independently of the result callback.
        threading.Thread(
            target=self._post_kick_sequence,
            args=(max(0.0, self._kick_timeout), self._walk_delay, self._stop_delay),
            daemon=True,
        ).start()
        return "kick sent"

    def _post_kick_sequence(self, kick_duration: float, walk_delay: float, stop_delay: float):
        clock = self.get_clock()

        clock.sleep_for(Duration(seconds=kick_duration))

        if walk_delay > 0.0:
            clock.sleep_for(Duration(seconds=walk_delay))

        self._cmd_vel_pub.publish(Twist())
        #self.get_logger().info("Post-kick: published walk command")

        if stop_delay > 0.0:
            clock.sleep_for(Duration(seconds=stop_delay))

        stop_twist = Twist()
        stop_twist.angular.x = -1.0
        self._cmd_vel_pub.publish(stop_twist)
        #self.get_logger().info("Post-kick: published full stop")

        with self._lock:
            self._kick_in_flight = False

    # ------------------------------------------------------------------
    # display
    # ------------------------------------------------------------------

    _STATE_LINES = 10

    def _render_state(self, last_key: str = "", info: str = ""):
        timeout_str = f"{self._kick_timeout:.1f}s" if self._kick_timeout >= 0 else "node default"
        mode_tag = "[angle]" if self._mode == "angle" else "[xy]   "
        state = (
            f"mode:        {mode_tag}              \n"
            f"angle:       {self._angle_deg:+.1f}°  (q/e, 5° steps)\n"
            f"x:           {self._x:+.4f}  (w/s)\n"
            f"y:           {self._y:+.4f}  (a/d)\n"
            f"strength:    {self._strength:.2f}  (r/f)\n"
            f"timeout:     {timeout_str}  (t/g)\n"
            f"walk delay:  {self._walk_delay:.1f}s  (z/h)\n"
            f"stop delay:  {self._stop_delay:.1f}s  (u/j)\n"
            f"in flight:   {'YES' if self._kick_in_flight else 'no '}    \n"
            f"last key:    '{last_key}'  {info:<30}"
        )
        sys.stdout.write("\x1b[A" * self._STATE_LINES)
        print(state)

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------

    def loop(self):
        print(HELP)
        print("\n" * self._STATE_LINES, end="")

        try:
            while True:
                info = ""
                key = self._get_key()

                if key == "q":
                    self._angle_deg = round(self._angle_deg - _STEP_ANGLE_DEG, 1)
                    self._mode = "angle"
                    self._apply_angle()
                elif key == "e":
                    self._angle_deg = round(self._angle_deg + _STEP_ANGLE_DEG, 1)
                    self._mode = "angle"
                    self._apply_angle()
                elif key == "w":
                    self._x = round(self._x + _STEP_XY, 3)
                    self._mode = "xy"
                    self._sync_angle_from_xy()
                elif key == "s":
                    self._x = round(self._x - _STEP_XY, 3)
                    self._mode = "xy"
                    self._sync_angle_from_xy()
                elif key == "a":
                    self._y = round(self._y + _STEP_XY, 3)
                    self._mode = "xy"
                    self._sync_angle_from_xy()
                elif key == "d":
                    self._y = round(self._y - _STEP_XY, 3)
                    self._mode = "xy"
                    self._sync_angle_from_xy()
                elif key == "r":
                    self._strength = round(self._strength + _STEP_STRENGTH, 2)
                elif key == "f":
                    self._strength = max(0.0, round(self._strength - _STEP_STRENGTH, 2))
                elif key == "t":
                    self._kick_timeout = round(self._kick_timeout + _STEP_TIMEOUT, 1)
                elif key == "g":
                    self._kick_timeout = round(self._kick_timeout - _STEP_TIMEOUT, 1)
                elif key == "z":
                    self._walk_delay = round(self._walk_delay + _STEP_DELAY, 1)
                elif key == "h":
                    self._walk_delay = max(0.0, round(self._walk_delay - _STEP_DELAY, 1))
                elif key == "u":
                    self._stop_delay = round(self._stop_delay + _STEP_DELAY, 1)
                elif key == "j":
                    self._stop_delay = max(0.0, round(self._stop_delay - _STEP_DELAY, 1))
                elif key in (" ", "\r", "\n"):
                    info = self._trigger_kick()
                elif key == "\x03":  # CTRL-C
                    break
                else:
                    info = f"unknown key '{key}'"

                self._render_state(key, info)

        except Exception as e:
            print(e)
        finally:
            stop = Twist()
            stop.angular.x = -1.0
            self._cmd_vel_pub.publish(stop)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)


if __name__ == "__main__":
    rclpy.init(args=None)
    node = KickTeleop()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()
