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
import tf2_geometry_msgs  # noqa: F401 -- registers the PointStamped transform used below
import tf2_ros
import transforms3d
from game_controller_hsl_interfaces.msg import GameState
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, Header

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
    ball_map: Stamped = field(default_factory=Stamped)  # filtered ball, transformed to map by us


def _fq(namespace: str, name: str) -> str:
    """Fully-qualified node name from a (namespace, name) pair, e.g. ('/', 'foo') -> '/foo'."""
    return f"{namespace.rstrip('/')}/{name}"


class _MonitorBase:
    """A rclpy context + node + executor bound to a single ROS domain, spun in a thread."""

    def __init__(self, domain: int, node_name: str):
        self.context = Context()
        self.context.init(domain_id=domain)
        self.node = Node(node_name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        # Graph counts (nodes/topics in this domain, excluding ourselves), refreshed on a timer.
        self.n_nodes: Optional[int] = None
        self.n_topics: Optional[int] = None

    @staticmethod
    def _store(slot: Stamped):
        return lambda msg: slot.set(msg)

    def refresh_counts(self) -> None:
        """Recount the nodes and topics in this domain, excluding our own monitoring node.

        A topic is skipped only if the sole participant is us; a node/topic that anyone else
        publishes, subscribes to, or hosts is counted.
        """
        node = self.node
        our = node.get_fully_qualified_name()
        try:
            self.n_nodes = sum(
                1 for name, ns in node.get_node_names_and_namespaces() if _fq(ns, name) != our
            )
            topics = 0
            for topic, _types in node.get_topic_names_and_types():
                endpoints = {
                    _fq(info.node_namespace, info.node_name)
                    for info in node.get_publishers_info_by_topic(topic)
                    + node.get_subscriptions_info_by_topic(topic)
                }
                if endpoints - {our}:  # someone other than us is on this topic
                    topics += 1
            self.n_topics = topics
        except Exception:  # noqa: BLE001 - graph queries can race with shutdown; keep last values
            pass

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

    def __init__(self, state: RobotState, map_frame: str = "map"):
        super().__init__(state.domain, f"robot_overview_probe_{state.domain}")
        self.state = state
        self.map_frame = map_frame
        self.node.create_subscription(Strategy, "strategy", self._store(state.strategy), 1)
        self.node.create_subscription(PoseWithCovarianceStamped, "pose_with_covariance", self._store(state.pose), 1)
        self.node.create_subscription(GameState, "gamestate", self._store(state.gamestate), 1)
        self.node.create_subscription(Float32, "time_to_ball", self._store(state.time_to_ball), 1)
        self.node.create_subscription(Twist, "cmd_vel", self._store(state.cmd_vel), 1)

        # The filtered ball is published in the odom frame; we hold this domain's TF tree
        # ourselves and transform each ball into the map frame so it is comparable to the
        # ground-truth ball. The listener feeds off this domain's /tf and /tf_static.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.node.create_subscription(
            PoseWithCovarianceStamped, "ball_position_relative_filtered", self._ball_filtered_cb, 1
        )

    def _ball_filtered_cb(self, msg: PoseWithCovarianceStamped) -> None:
        # Stamp 0 -> use the latest available transform (avoids sim-time extrapolation issues).
        point = PointStamped(
            header=Header(stamp=Time().to_msg(), frame_id=msg.header.frame_id),
            point=msg.pose.pose.position,
        )
        try:
            self.state.ball_map.set(self.tf_buffer.transform(point, self.map_frame, timeout=Duration()))
        except tf2_ros.TransformException:
            pass  # TF not ready yet / frame missing — keep the previous value


class GroundTruthMonitor(_MonitorBase):
    """Reads the sim's ground truth from the main domain (never bridged): per-robot
    ``robotN/true_pose`` and the single global ``true_ball``."""

    def __init__(self, states: list[RobotState], main_domain: int):
        super().__init__(main_domain, "robot_overview_truth")
        self.true_ball = Stamped()  # single global ball, shared across robots
        self.node.create_subscription(PointStamped, "true_ball", self._store(self.true_ball), 1)
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


def _fmt_error(est: Optional[tuple[float, float, float]], true: Optional[tuple[float, float, float]]) -> str:
    if est is None or true is None:
        return "     -     "
    dist = math.hypot(est[0] - true[0], est[1] - true[1])
    # wrap yaw error into (-180, 180]
    yaw_err = math.degrees((est[2] - true[2] + math.pi) % (2 * math.pi) - math.pi)
    return f"{dist:4.2f}m {yaw_err:+4.0f}°"


def _ball_xy(msg: Optional[PointStamped]) -> Optional[tuple[float, float]]:
    return None if msg is None else (msg.point.x, msg.point.y)


def _fmt_dist(a: Optional[tuple[float, float]], b: Optional[tuple[float, float]]) -> str:
    if a is None or b is None:
        return "   -  "
    return f"{math.hypot(a[0] - b[0], a[1] - b[1]):5.2f}m"


def _fmt_age(age: Optional[float]) -> str:
    if age is None:
        return "  -  "
    if age > 5.0:
        return f"{age:4.0f}s"  # clearly stale
    return f"{age:4.1f}s"


def render(
    domain_monitors: list["DomainMonitor"],
    true_ball: Optional[tuple[float, float]] = None,
    rotate_180: bool = True,
) -> str:
    # true_ball is the sim ground-truth ball (map frame). ball_err is the distance between
    # the filtered ball (transformed to map) and the ground-truth ball (position only).
    # nodes/topics count the other participants discovered in each robot's domain.
    header = (
        f"{'robot':>6} | {'role':>8} | {'action':>15} | {'pose err (m,°)':^14} | "
        f"{'ball err':>8} | {'ttb':>6} | {'game':>8} | {'pen':>3} | {'nds':>4} | {'tpc':>4} | {'age':>5}"
    )
    lines = [header, "-" * len(header)]
    for m in domain_monitors:
        s = m.state
        strat: Optional[Strategy] = s.strategy.value
        gs: Optional[GameState] = s.gamestate.value
        ttb: Optional[Float32] = s.time_to_ball.value
        est = _xy_yaw(s.pose.value)
        true = _xy_yaw(s.true_pose.value)
        if rotate_180:
            true = _rotate_180_about_origin(true)

        role = ROLE.get(strat.role, "?") if strat else "-"
        action = ACTION.get(strat.action, "?") if strat else "-"
        ball_err = _fmt_dist(_ball_xy(s.ball_map.value), true_ball)
        ttb_str = f"{ttb.data:5.1f}s" if ttb else "   -  "
        game = GAME_STATE.get(gs.main_state, "?") if gs else "-"
        pen = ("Y" if gs.penalized else "n") if gs and hasattr(gs, "penalized") else "-"
        nds = "-" if m.n_nodes is None else str(m.n_nodes)
        tpc = "-" if m.n_topics is None else str(m.n_topics)
        # freshest signal we have from this robot -> its liveness
        ages = [a for a in (s.strategy.age(), s.pose.age(), s.gamestate.age()) if a is not None]
        age = _fmt_age(min(ages) if ages else None)

        lines.append(
            f"{s.domain:>6} | {role:>8} | {action:>15} | {_fmt_error(est, true):^14} | "
            f"{ball_err:>8} | {ttb_str:>6} | {game:>8} | {pen:>3} | {nds:>4} | {tpc:>4} | {age:>5}"
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
        "--count-interval",
        type=float,
        default=3.0,
        help="How often (seconds) to recount nodes/topics per domain (default: 3)",
    )
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
    domain_monitors = [DomainMonitor(s) for s in states]
    truth = GroundTruthMonitor(states, args.main_domain)
    monitors: list[_MonitorBase] = [*domain_monitors, truth]
    for m in monitors:
        m.start()

    def true_ball_xy() -> Optional[tuple[float, float]]:
        xy = _ball_xy(truth.true_ball.value)
        if xy is None or not args.rotate_180:
            return xy
        return (-xy[0], -xy[1])  # same 180° realignment as the ground-truth pose

    last_count = 0.0

    def maybe_refresh_counts() -> None:
        nonlocal last_count
        now = time.monotonic()
        if now - last_count >= args.count_interval:
            for m in monitors:
                m.refresh_counts()
            last_count = now

    def global_line() -> str:
        nds = "-" if truth.n_nodes is None else truth.n_nodes
        tpc = "-" if truth.n_topics is None else truth.n_topics
        return f"global namespace (domain {args.main_domain}):  nodes={nds}  topics={tpc}"

    try:
        if args.once:
            time.sleep(1.0)  # let subscriptions match and receive
            maybe_refresh_counts()
            print(render(domain_monitors, true_ball_xy(), args.rotate_180))
            print("\n" + global_line())
        else:
            period = 1.0 / args.rate
            while True:
                maybe_refresh_counts()
                # clear screen + home cursor, then redraw the table in place
                print("\033[2J\033[H", end="")
                rot = "  (truth rotated 180°)" if args.rotate_180 else ""
                print(
                    f"Robot overview  domains={domains}  truth@{args.main_domain}{rot}  "
                    f"({time.strftime('%H:%M:%S')})\n"
                    "ball err = filtered ball (transformed odom->map) vs true ball | "
                    "nds/tpc = other nodes/topics in domain\n"
                )
                print(render(domain_monitors, true_ball_xy(), args.rotate_180))
                print("\n" + global_line())
                time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        for m in monitors:
            m.shutdown()


if __name__ == "__main__":
    main()
