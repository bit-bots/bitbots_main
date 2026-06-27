import math
from typing import Optional

import numpy as np
from geometry_msgs.msg import Twist
from soccer_vision_3d_msgs.msg import BallArray
from std_msgs.msg import Bool

from handlers.handler import Handler


class SoccerCommandHandler(Handler):
    """Builds the soccer policy's command terms from the bitbots topic world.

    The encdec soccer policy consumes the command as TWO observation terms that
    are both derived from one 7-D snapshot per control step
    ``[vx, vy, wz, ball_x_b, ball_y_b, kick_dir_b, kick_speed]`` (robot yaw-base
    frame):

      * ``soccer_command``   : the 7-D snapshot with the ball columns [3:5]
        deleted -> 5-D ``[vx, vy, wz, kick_dir, kick_speed]``, kept as a
        ``history_samples`` history pushed EVERY step (50 Hz "clean" history).
      * ``ball_pos_b_noisy`` : the ball columns [3:5] -> 2-D ``[ball_x, ball_y]``,
        kept as a ``history_samples`` history pushed only every ``pub_period``
        steps (the 10 Hz async ball history fed to the encoder).

    Wiring (chosen for the bitbots stack):
      * velocity ``[vx, vy, wz]`` comes from ``cmd_vel``.
      * the kick is triggered by ``rl_kick_active`` (Bool). While kicking, the
        velocity is forced to 0 (as in training), the ball offset comes from
        ``balls_relative`` (nearest ball, base-frame xy), and the kick direction
        follows the ``cmd_vel`` heading once it leaves a deadzone, with a constant
        ``kick_speed``. When not kicking the ball is masked to 0, matching the
        walk role the policy was trained with.
    """

    def __init__(self, node):
        self._node = node

        self._kick_speed_const = float(node.get_parameter("command.kick_speed").value)
        self._deadzone = float(node.get_parameter("command.deadzone").value)
        self._pub_period = max(1, int(node.get_parameter("command.pub_period").value))
        self._history_samples = max(1, int(node.get_parameter("command.history_samples").value))

        self._cmd_vel: Optional[Twist] = None
        self._kick_active = False
        self._ball_pos: Optional[np.ndarray] = None

        self._node.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 1)
        self._node.create_subscription(Bool, "rl_kick_active", self._kick_callback, 1)
        self._node.create_subscription(BallArray, "balls_relative", self._ball_callback, 1)

        # history state (newest at index -1), lazily initialized on first update
        self._clean_hist: Optional[np.ndarray] = None  # (H, 7), pushed every step
        self._soccer_hist: Optional[np.ndarray] = None  # (H, 7), pushed every pub_period
        self._pub_counter = 0
        self._last_kick_dir = 0.0

    # ------------------------------------------------------------------ #
    # subscriptions
    # ------------------------------------------------------------------ #
    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._cmd_vel = msg

    def _kick_callback(self, msg: Bool) -> None:
        self._kick_active = msg.data

    def _ball_callback(self, msg: BallArray) -> None:
        if msg.balls:
            center = msg.balls[0].center
            self._ball_pos = np.array([center.x, center.y], dtype=np.float32)

    def has_data(self) -> bool:
        # Non-blocking: missing cmd_vel/ball just yield zeros, so the policy can
        # run (standing) before any command or ball is received.
        return True

    # ------------------------------------------------------------------ #
    # per-step command construction
    # ------------------------------------------------------------------ #
    def _build_soccer_cmd(self) -> np.ndarray:
        if self._cmd_vel is not None:
            cmd_vx = float(self._cmd_vel.linear.x)
            cmd_vy = float(self._cmd_vel.linear.y)
            cmd_wz = float(self._cmd_vel.angular.z)
        else:
            cmd_vx = cmd_vy = cmd_wz = 0.0

        if self._kick_active:
            vx, vy, wz = 0.0, 0.0, 0.0
            if self._ball_pos is not None:
                ball_x, ball_y = float(self._ball_pos[0]), float(self._ball_pos[1])
            else:
                ball_x, ball_y = 0.0, 0.0
            if math.hypot(cmd_vx, cmd_vy) >= self._deadzone:
                self._last_kick_dir = math.atan2(cmd_vy, cmd_vx)
                kick_speed = self._kick_speed_const
            else:
                kick_speed = 0.0
            kick_dir = self._last_kick_dir
        else:
            vx, vy, wz = cmd_vx, cmd_vy, cmd_wz
            ball_x, ball_y = 0.0, 0.0
            kick_dir = 0.0
            kick_speed = 0.0
            self._last_kick_dir = 0.0

        return np.array([vx, vy, wz, ball_x, ball_y, kick_dir, kick_speed], dtype=np.float32)

    def update(self) -> None:
        """Advance both history buffers by one control step. Call once per step."""
        cmd7 = self._build_soccer_cmd()

        if self._clean_hist is None:
            self._clean_hist = np.tile(cmd7, (self._history_samples, 1)).astype(np.float32)
            self._soccer_hist = np.tile(cmd7, (self._history_samples, 1)).astype(np.float32)
            self._pub_counter = 0

        # vel/kick (cols 0:3, 5:7): 50 Hz real history, shift+push every step
        self._clean_hist[:-1] = self._clean_hist[1:]
        self._clean_hist[-1] = cmd7

        # ball xy (cols 3:5): 10 Hz async history, shift+push every pub_period steps
        if self._pub_counter % self._pub_period == 0:
            self._soccer_hist[:-1] = self._soccer_hist[1:]
            self._soccer_hist[-1] = cmd7
        self._pub_counter += 1

    def get_soccer_command_obs(self) -> np.ndarray:
        """5-D-per-frame command history (ball columns deleted), flattened."""
        assert self._clean_hist is not None, "update() must be called before reading the command"
        out = np.delete(self._clean_hist, [3, 4], axis=1)  # (H, 5)
        return out.reshape(-1).copy()

    def get_ball_obs(self) -> np.ndarray:
        """Ball xy history fed to the encoder, flattened."""
        assert self._soccer_hist is not None, "update() must be called before reading the ball"
        return self._soccer_hist[:, 3:5].reshape(-1).copy()
