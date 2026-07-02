#!/usr/bin/env python3
"""Overview of robot states across multiple ROS domains in the MuJoCo multi-robot sim.

Each robot's teamplayer stack runs in its own ROS_DOMAIN_ID (11, 12, 13, ... see
``mujoco_simulation.launch.py``). A single ROS process normally only sees one domain,
so this script creates one independent ``rclpy.Context`` per domain, each initialized
with its own ``domain_id`` and spun in its own thread. The main thread periodically
prints a refreshing overview table.

Self-contained: source the workspace (for the message types) and run e.g.::

    python3 scripts/robot_overview.py --domains 11 12 13 14
    python3 scripts/robot_overview.py --num-robots 4        # -> domains 11..14
    python3 scripts/robot_overview.py --once                # single snapshot, then exit

Per-robot state topics are read at the domain root (the ``robotN/`` namespace only
exists in the main domain; inside a robot's own domain everything is at ``/``). The
ground-truth pose is the exception: the sim publishes ``robotN/true_pose`` only in the
main domain (never bridged into a robot's domain), so it is read there and compared
against each robot's localized ``pose_with_covariance`` to show the localization error.
"""

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
import transforms3d
from game_controller_hsl_interfaces.msg import GameState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

from bitbots_msgs.msg import Strategy

# ---- Enum -> label maps (kept in sync with the .msg definitions) ------------------

ROLE = {
    Strategy.ROLE_UNDEFINED: "undef",
    Strategy.ROLE_IDLING: "idling",
    Strategy.ROLE_OTHER: "other",
    Strategy.ROLE_STRIKER: "striker",
    Strategy.ROLE_SUPPORTER: "support",
    Strategy.ROLE_DEFENDER: "defender",
    Strategy.ROLE_GOALIE: "goalie",
}

ACTION = {
    Strategy.ACTION_UNDEFINED: "undef",
    Strategy.ACTION_POSITIONING: "positioning",
    Strategy.ACTION_GOING_TO_BALL: "going_to_ball",
    Strategy.ACTION_TRYING_TO_SCORE: "trying_to_score",
    Strategy.ACTION_WAITING: "waiting",
    Strategy.ACTION_KICKING: "kicking",
    Strategy.ACTION_SEARCHING: "searching",
    Strategy.ACTION_LOCALIZING: "localizing",
    Strategy.ACTION_PASSIVE: "passive",
}

GAME_STATE = {
    GameState.STATE_INITIAL: "INITIAL",
    GameState.STATE_READY: "READY",
    GameState.STATE_SET: "SET",
    GameState.STATE_PLAYING: "PLAYING",
    GameState.STATE_FINISHED: "FINISHED",
}


@dataclass
class Stamped:
    """A latest-value slot with the wall-clock time it arrived (for staleness)."""

    value: object = None
    at: float = 0.0

    def set(self, value):
        self.value = value
        self.at = time.monotonic()

    def age(self) -> Optional[float]:
        return None if self.value is None else time.monotonic() - self.at


@dataclass
class RobotState:
    """Latest cached messages for one robot domain. Callbacks only write here."""

    domain: int
    strategy: Stamped = field(default_factory=Stamped)
    pose: Stamped = field(default_factory=Stamped)  # localization estimate (robot domain)
    gamestate: Stamped = field(default_factory=Stamped)
    time_to_ball: Stamped = field(default_factory=Stamped)
    cmd_vel: Stamped = field(default_factory=Stamped)
    true_pose: Stamped = field(default_factory=Stamped)  # ground truth (main domain)


class _MonitorBase:
    """A rclpy context + node + executor bound to a single ROS domain, spun in a thread."""

    def __init__(self, domain: int, node_name: str):
        self.context = Context()
        self.context.init(domain_id=domain)
        self.node = Node(node_name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)

    @staticmethod
    def _store(slot: Stamped):
        return lambda msg: slot.set(msg)

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # noqa: BLE001 - context shutdown races on exit are expected
            pass

    def start(self):
        self._thread.start()

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.try_shutdown()


class DomainMonitor(_MonitorBase):
    """Reads one robot's own-domain state topics into the given shared RobotState."""

    def __init__(self, state: RobotState):
        super().__init__(state.domain, f"robot_overview_probe_{state.domain}")
        self.state = state
        self.node.create_subscription(Strategy, "strategy", self._store(state.strategy), 1)
        self.node.create_subscription(PoseWithCovarianceStamped, "pose_with_covariance", self._store(state.pose), 1)
        self.node.create_subscription(GameState, "gamestate", self._store(state.gamestate), 1)
        self.node.create_subscription(Float32, "time_to_ball", self._store(state.time_to_ball), 1)
        self.node.create_subscription(Twist, "cmd_vel", self._store(state.cmd_vel), 1)


class GroundTruthMonitor(_MonitorBase):
    """Reads every robot's ``robotN/true_pose`` from the main domain (never bridged)."""

    def __init__(self, states: list[RobotState], main_domain: int):
        super().__init__(main_domain, "robot_overview_truth")
        for state in states:
            self.node.create_subscription(
                PoseStamped, f"robot{state.domain}/true_pose", self._store(state.true_pose), 1
            )


# ---- Rendering --------------------------------------------------------------------


def _xy_yaw(msg) -> Optional[tuple[float, float, float]]:
    """Extract (x, y, yaw) from a PoseStamped or PoseWithCovarianceStamped, in radians."""
    if msg is None:
        return None
    pose = msg.pose.pose if isinstance(msg, PoseWithCovarianceStamped) else msg.pose
    q = pose.orientation
    yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]
    return (pose.position.x, pose.position.y, yaw)


def _rotate_180_about_origin(pose: Optional[tuple[float, float, float]]) -> Optional[tuple[float, float, float]]:
    """Rotate an (x, y, yaw) pose 180° about the field origin.

    The kid-size field is point-symmetric about its center, so a 180° rotation
    (x, y, yaw) -> (-x, -y, yaw + 180°) maps a pose onto a physically equivalent one.
    We use it to bring the sim's ground-truth (MuJoCo world) frame into the same
    orientation as the localization ``map`` frame, which is rotated 180° relative to it.
    """
    if pose is None:
        return None
    x, y, yaw = pose
    return (-x, -y, (yaw + 2 * math.pi) % (2 * math.pi) - math.pi)  # == wrap(yaw + 180°)


def _fmt_pose(pose: Optional[tuple[float, float, float]]) -> str:
    if pose is None:
        return "       -        "
    x, y, yaw = pose
    return f"({x:+5.2f},{y:+5.2f}) {math.degrees(yaw):+4.0f}°"


def _fmt_error(est: Optional[tuple[float, float, float]], true: Optional[tuple[float, float, float]]) -> str:
    if est is None or true is None:
        return "     -     "
    dist = math.hypot(est[0] - true[0], est[1] - true[1])
    # wrap yaw error into (-180, 180]
    yaw_err = math.degrees((est[2] - true[2] + math.pi) % (2 * math.pi) - math.pi)
    return f"{dist:4.2f}m {yaw_err:+4.0f}°"


def _fmt_age(age: Optional[float]) -> str:
    if age is None:
        return "  -  "
    if age > 5.0:
        return f"{age:4.0f}s"  # clearly stale
    return f"{age:4.1f}s"


def render(states: list[RobotState], rotate_180: bool = True) -> str:
    header = (
        f"{'robot':>6} | {'role':>8} | {'action':>15} | {'est (x,y) yaw':^17} | "
        f"{'true (x,y) yaw':^17} | {'err (m,°)':^11} | {'ttb':>6} | {'game':>8} | {'pen':>3} | {'age':>5}"
    )
    lines = [header, "-" * len(header)]
    for s in states:
        strat: Optional[Strategy] = s.strategy.value
        gs: Optional[GameState] = s.gamestate.value
        ttb: Optional[Float32] = s.time_to_ball.value
        est = _xy_yaw(s.pose.value)
        true = _xy_yaw(s.true_pose.value)
        if rotate_180:
            true = _rotate_180_about_origin(true)

        role = ROLE.get(strat.role, "?") if strat else "-"
        action = ACTION.get(strat.action, "?") if strat else "-"
        ttb_str = f"{ttb.data:5.1f}s" if ttb else "   -  "
        game = GAME_STATE.get(gs.main_state, "?") if gs else "-"
        pen = ("Y" if gs.penalized else "n") if gs and hasattr(gs, "penalized") else "-"
        # freshest signal we have from this robot -> its liveness
        ages = [a for a in (s.strategy.age(), s.pose.age(), s.gamestate.age()) if a is not None]
        age = _fmt_age(min(ages) if ages else None)

        lines.append(
            f"{s.domain:>6} | {role:>8} | {action:>15} | {_fmt_pose(est):^17} | "
            f"{_fmt_pose(true):^17} | {_fmt_error(est, true):^11} | "
            f"{ttb_str:>6} | {game:>8} | {pen:>3} | {age:>5}"
        )
    return "\n".join(lines)


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--domains", type=int, nargs="+", help="Explicit ROS_DOMAIN_IDs to monitor, e.g. 11 12 13 14")
    g.add_argument("--num-robots", type=int, help="Monitor domains 11 .. 11+N-1")
    ap.add_argument("--rate", type=float, default=2.0, help="Refresh rate in Hz (default: 2)")
    ap.add_argument(
        "--main-domain",
        type=int,
        default=int(os.getenv("ROS_DOMAIN_ID", "0")),
        help="Domain the sim runs in, where robotN/true_pose is published (default: $ROS_DOMAIN_ID or 0)",
    )
    ap.add_argument("--once", action="store_true", help="Collect briefly, print one snapshot, then exit")
    ap.add_argument(
        "--rotate-180",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Rotate the ground-truth pose 180° about the origin to match the localization "
        "map frame (the field is symmetric, so this only realigns the frames). Use "
        "--no-rotate-180 if your sim world and map frame are already aligned.",
    )
    return ap.parse_args()


def main():
    args = parse_args()
    if args.domains:
        domains = args.domains
    elif args.num_robots:
        domains = list(range(11, 11 + args.num_robots))
    else:
        domains = [11, 12, 13, 14]

    states = [RobotState(domain=d) for d in domains]
    monitors: list[_MonitorBase] = [DomainMonitor(s) for s in states]
    monitors.append(GroundTruthMonitor(states, args.main_domain))
    for m in monitors:
        m.start()

    try:
        if args.once:
            time.sleep(1.0)  # let subscriptions match and receive
            print(render(states, args.rotate_180))
        else:
            period = 1.0 / args.rate
            while True:
                # clear screen + home cursor, then redraw the table in place
                print("\033[2J\033[H", end="")
                rot = "  (truth rotated 180°)" if args.rotate_180 else ""
                print(
                    f"Robot overview  domains={domains}  truth@{args.main_domain}{rot}  "
                    f"({time.strftime('%H:%M:%S')})\n"
                )
                print(render(states, args.rotate_180))
                time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        for m in monitors:
            m.shutdown()


if __name__ == "__main__":
    main()
