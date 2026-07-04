#!/usr/bin/env python3
"""
Interactive visualization / test harness for ``PathfindingCapsule.get_ball_goal``.

This calls the *real* ``PathfindingCapsule.get_ball_goal`` (see
``bitbots_blackboard/capsules/pathfinding_capsule.py``). The capsule is built
without running its ``__init__`` (no ROS node) and given a stubbed blackboard
whose ``world_model`` / ``costmap`` / ``tf_buffer`` read the interactive robot
and ball poses. Move them around and the goal pose the actual function returns
(and, for ``rl_kick``, the approach arc) is drawn live.

Coordinate frame (matches the drawing request):
  * +x points towards the left (opponent / left goal)
  * +y points down
  * +z points towards the viewer (right-handed)

The field lines from ``small_devision_2026/lines_bw.png`` are drawn in the
background, spanning x in [-5.5, 5.5] and y in [-4, 4].

Interaction:
  * Drag the ROBOT body (blue) to move it.
  * Drag the ROBOT heading handle (the tip of the blue arrow) to rotate it.
  * Drag the BALL (orange) to move it.
  * Use the radio buttons to switch the ``BallGoalType``.

Run:
    python3 visualize_get_ball_goal.py
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from bitbots_blackboard.capsules.pathfinding_capsule import BallGoalType, PathfindingCapsule
from matplotlib.patches import Arc, Circle, FancyArrow
from matplotlib.widgets import RadioButtons
from ros2_numpy import numpify
from tf_transformations import euler_from_quaternion

# --------------------------------------------------------------------------- #
# Field / behavior parameters (mirrors config, so no ROS param server needed). #
# --------------------------------------------------------------------------- #

FIELD_LENGTH = 9.0  # field.size.x
GOAL_WIDTH = 1.8  # field.goal.width

# body_behavior.yaml values
BALL_APPROACH_DIST = 0.2
BALL_APPROACH_SIDE_OFFSET = 0.0

RL_KICK = {
    "approach_dist": 0.3,
    "tolerance_dist": 0.5,
    "approach_arc_half_degree": 150.0,
    "goal_line_offset": 0.25,
    "max_approach_angle_deg": 90.0,
}

# Background image extent (data coordinates).
IMG_PATH = (
    Path(__file__).resolve().parents[3]  # .../src
    / "bitbots_misc/bitbots_parameter_blackboard/config/fields/small_devision_2026/lines_bw.png"
)
IMG_EXTENT = (-5.5, 5.5, -4.0, 4.0)  # (xmin, xmax, ymin, ymax)


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # heading, only meaningful for the robot


MAP_FRAME = "map"
BASE_FOOTPRINT_FRAME = "base_footprint"


# --------------------------------------------------------------------------- #
# Stubbed blackboard: the real get_ball_goal reads robot/ball poses and the    #
# field/config through these objects, so we only override those accessors.     #
# --------------------------------------------------------------------------- #


class FakeWorldModel:
    def __init__(self, robot: Pose, ball: Pose):
        self.robot = robot
        self.ball = ball
        self.goal_width = GOAL_WIDTH
        self.field_length = FIELD_LENGTH
        self.base_footprint_frame = BASE_FOOTPRINT_FRAME

    def get_ball_position_xy(self):
        return self.ball.x, self.ball.y

    def get_current_position(self):
        return self.robot.x, self.robot.y, self.robot.theta

    def get_map_based_opp_goal_center_xy(self):
        return self.field_length / 2, 0.0

    def get_map_based_opp_goal_angle_from_ball(self):
        gx, gy = self.get_map_based_opp_goal_center_xy()
        return math.atan2(gy - self.ball.y, gx - self.ball.x)

    def get_ball_position_uv(self):
        # ball expressed in the robot (base_footprint) frame
        dx, dy = self.ball.x - self.robot.x, self.ball.y - self.robot.y
        c, s = math.cos(-self.robot.theta), math.sin(-self.robot.theta)
        return c * dx - s * dy, s * dx + c * dy


class FakeCostmap:
    def __init__(self, world_model: FakeWorldModel):
        self.wm = world_model

    def get_gradient_direction_at_field_position(self, _x, _y):
        # No real costmap available; fall back to the direction to the opp goal.
        return self.wm.get_map_based_opp_goal_angle_from_ball()


class FakeTfBuffer:
    """Minimal stand-in for tf2: identity for map->map, robot pose for bf->map."""

    def __init__(self, world_model: FakeWorldModel):
        self.wm = world_model

    def transform(self, pose_stamped, target_frame):
        from bitbots_utils.transforms import quat_from_yaw
        from geometry_msgs.msg import Point, PoseStamped

        src = pose_stamped.header.frame_id
        px = pose_stamped.pose.position.x
        py = pose_stamped.pose.position.y
        yaw = euler_from_quaternion(numpify(pose_stamped.pose.orientation))[2]

        if src == target_frame:
            mx, my, myaw = px, py, yaw
        elif src == self.wm.base_footprint_frame and target_frame == MAP_FRAME:
            r = self.wm.robot
            c, s = math.cos(r.theta), math.sin(r.theta)
            mx = r.x + c * px - s * py
            my = r.y + s * px + c * py
            myaw = r.theta + yaw
        else:
            raise ValueError(f"FakeTfBuffer cannot transform {src} -> {target_frame}")

        out = PoseStamped()
        out.header.frame_id = target_frame
        out.pose.position = Point(x=mx, y=my, z=0.0)
        out.pose.orientation = quat_from_yaw(myaw)
        return out


class FakeBlackboard:
    def __init__(self, robot: Pose, ball: Pose):
        self.world_model = FakeWorldModel(robot, ball)
        self.costmap = FakeCostmap(self.world_model)
        self.tf_buffer = FakeTfBuffer(self.world_model)
        self.map_frame = MAP_FRAME
        self.config = {
            "ball_approach_dist": BALL_APPROACH_DIST,
            "rl_kick": RL_KICK,
        }


def make_capsule(robot: Pose, ball: Pose) -> PathfindingCapsule:
    """Build the real capsule without running __init__ (which needs a ROS node)."""
    capsule = PathfindingCapsule.__new__(PathfindingCapsule)
    capsule._blackboard = FakeBlackboard(robot, ball)
    return capsule


# --------------------------------------------------------------------------- #
# Interactive figure.                                                          #
# --------------------------------------------------------------------------- #

ARROW_LEN = 0.6
PICK_RADIUS = 0.35


class Visualizer:
    def __init__(self):
        self.robot = Pose(x=-2.0, y=1.0, theta=math.radians(20))
        self.ball = Pose(x=0.5, y=-0.5)
        self.target = BallGoalType.RL_KICK
        self._drag = None  # one of: "robot", "robot_rot", "ball"

        # Real capsule; its stub blackboard reads the poses above by reference.
        self.capsule = make_capsule(self.robot, self.ball)

        self.fig, self.ax = plt.subplots(figsize=(11, 8))
        self.fig.subplots_adjust(left=0.20, right=0.98, top=0.98, bottom=0.05)

        # Background field image.
        if IMG_PATH.exists():
            img = mpimg.imread(IMG_PATH)
            self.ax.imshow(img, extent=IMG_EXTENT, origin="upper", cmap="gray", zorder=0)
        else:
            print(f"WARNING: field image not found at {IMG_PATH}")

        # x towards left, y down.
        self.ax.set_xlim(5.5, -5.5)
        self.ax.set_ylim(4.0, -4.0)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x (towards left goal)")
        self.ax.set_ylabel("y (down)")

        # Radio buttons to pick the target type.
        rax = self.fig.add_axes([0.01, 0.5, 0.15, 0.18])
        self._targets = list(BallGoalType)
        self.radio = RadioButtons(rax, [t.value for t in self._targets], active=self._targets.index(self.target))
        self.radio.on_clicked(self._on_target)

        # Static artists (updated in redraw).
        self._dynamic = []

        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)

        self.redraw()

    # -- helpers ---------------------------------------------------------- #
    def _robot_tip(self):
        return (
            self.robot.x + math.cos(self.robot.theta) * ARROW_LEN,
            self.robot.y + math.sin(self.robot.theta) * ARROW_LEN,
        )

    @staticmethod
    def _dist(ax, ay, bx, by):
        return math.hypot(ax - bx, ay - by)

    # -- events ----------------------------------------------------------- #
    def _on_target(self, label):
        self.target = BallGoalType(label)
        self.redraw()

    def _on_press(self, event):
        if event.inaxes is not self.ax or event.xdata is None:
            return
        px, py = event.xdata, event.ydata
        tip_x, tip_y = self._robot_tip()
        # Rotation handle wins over body (it sits on the arrow tip).
        if self._dist(px, py, tip_x, tip_y) < PICK_RADIUS:
            self._drag = "robot_rot"
        elif self._dist(px, py, self.robot.x, self.robot.y) < PICK_RADIUS:
            self._drag = "robot"
        elif self._dist(px, py, self.ball.x, self.ball.y) < PICK_RADIUS:
            self._drag = "ball"

    def _on_release(self, _event):
        self._drag = None

    def _on_motion(self, event):
        if self._drag is None or event.inaxes is not self.ax or event.xdata is None:
            return
        px, py = event.xdata, event.ydata
        if self._drag == "robot":
            self.robot.x, self.robot.y = px, py
        elif self._drag == "robot_rot":
            self.robot.theta = math.atan2(py - self.robot.y, px - self.robot.x)
        elif self._drag == "ball":
            self.ball.x, self.ball.y = px, py
        self.redraw()

    # -- drawing ---------------------------------------------------------- #
    def _arrow(self, x, y, theta, length, color, zorder=5):
        return self.ax.add_patch(
            FancyArrow(
                x,
                y,
                math.cos(theta) * length,
                math.sin(theta) * length,
                width=0.03,
                head_width=0.15,
                head_length=0.15,
                length_includes_head=True,
                color=color,
                zorder=zorder,
            )
        )

    def redraw(self):
        for art in self._dynamic:
            art.remove()
        self._dynamic = []

        # --- opponent (left) goal reference ---
        gx = FIELD_LENGTH / 2
        for sign in (-1, 1):
            self._dynamic.append(
                self.ax.plot(
                    [gx, gx], [sign * GOAL_WIDTH / 2, sign * GOAL_WIDTH / 2], marker="s", color="red", zorder=3
                )[0]
            )
        self._dynamic.append(
            self.ax.plot([gx, gx], [-GOAL_WIDTH / 2, GOAL_WIDTH / 2], color="red", lw=2, zorder=3, label="left goal")[0]
        )

        # --- ball ---
        ball_c = Circle((self.ball.x, self.ball.y), 0.13, color="orange", zorder=6, ec="k")
        self.ax.add_patch(ball_c)
        self._dynamic.append(ball_c)

        # --- robot ---
        robot_c = Circle((self.robot.x, self.robot.y), 0.15, color="royalblue", zorder=6, ec="k")
        self.ax.add_patch(robot_c)
        self._dynamic.append(robot_c)
        self._dynamic.append(self._arrow(self.robot.x, self.robot.y, self.robot.theta, ARROW_LEN, "royalblue"))
        tip_x, tip_y = self._robot_tip()
        tip_c = Circle((tip_x, tip_y), 0.08, color="navy", zorder=7)
        self.ax.add_patch(tip_c)
        self._dynamic.append(tip_c)

        # --- compute + draw goal (calls the real capsule method) ---
        pose = self.capsule.get_ball_goal(self.target, BALL_APPROACH_DIST, BALL_APPROACH_SIDE_OFFSET)
        gx_ = pose.pose.position.x
        gy_ = pose.pose.position.y
        gth_ = euler_from_quaternion(numpify(pose.pose.orientation))[2]
        goal_c = Circle((gx_, gy_), 0.12, color="lime", zorder=8, ec="k")
        self.ax.add_patch(goal_c)
        self._dynamic.append(goal_c)
        self._dynamic.append(self._arrow(gx_, gy_, gth_, ARROW_LEN, "green", zorder=8))

        # --- rl_kick arc visualization (drawing only; mirrors the branch) ---
        if self.target == BallGoalType.RL_KICK:
            half = math.radians(RL_KICK["approach_arc_half_degree"])
            approach = RL_KICK["approach_dist"]
            max_app = math.radians(RL_KICK["max_approach_angle_deg"])
            # aim point: goal center, shifted 2*goal_line_offset beyond the goal line
            tgt = (FIELD_LENGTH / 2 + 2 * RL_KICK["goal_line_offset"], 0.0)
            angle_to_goal = math.atan2(self.ball.y - tgt[1], self.ball.x - tgt[0])
            arc_start = angle_to_goal - half
            arc_end = angle_to_goal + half
            in_arc = self.capsule.is_point_in_arc(
                self.robot.x, self.robot.y, self.ball.x, self.ball.y, approach, arc_start, arc_end
            )
            # kick arc: radius=approach_dist (used for the in-arc / hysteresis check)
            kick_arc = Arc(
                (self.ball.x, self.ball.y),
                2 * approach,
                2 * approach,
                angle=0,
                theta1=math.degrees(arc_start),
                theta2=math.degrees(arc_end),
                color="purple",
                lw=2,
                zorder=4,
            )
            self.ax.add_patch(kick_arc)
            self._dynamic.append(kick_arc)
            # approach clamp cone: allowed approach-point directions (behind the
            # ball, within +/- max_approach_angle of straight-away-from-goal)
            behind = angle_to_goal + math.pi
            for edge in (behind - max_app, behind + max_app):
                self._dynamic.append(
                    self.ax.plot(
                        [self.ball.x, self.ball.x + math.cos(edge) * approach * 1.3],
                        [self.ball.y, self.ball.y + math.sin(edge) * approach * 1.3],
                        "--",
                        color="teal",
                        lw=1.5,
                        zorder=4,
                    )[0]
                )
            # aim line: ball -> goal target point
            self._dynamic.append(
                self.ax.plot([self.ball.x, tgt[0]], [self.ball.y, tgt[1]], ":", color="purple", zorder=3)[0]
            )
            state = "IN ARC (kick!)" if in_arc else "approaching"
            title = f"rl_kick — robot {state}"
        else:
            title = f"target = {self.target.value}"

        self.ax.set_title(
            f"{title}\nrobot=({self.robot.x:.2f}, {self.robot.y:.2f}, "
            f"{math.degrees(self.robot.theta):.0f}°)  ball=({self.ball.x:.2f}, {self.ball.y:.2f})  "
            f"goal=({gx_:.2f}, {gy_:.2f}, {math.degrees(gth_):.0f}°)",
            fontsize=9,
        )
        self.fig.canvas.draw_idle()


def main():
    viz = Visualizer()
    if os.environ.get("HEADLESS"):
        # Smoke test: render one frame to disk instead of showing a window.
        out = Path(os.environ.get("HEADLESS_OUT", "/tmp/get_ball_goal.png"))
        viz.fig.savefig(out, dpi=90)
        print(f"saved {out}")
        return
    plt.show()


if __name__ == "__main__":
    main()
