import random
from dataclasses import dataclass
from enum import StrEnum
from typing import TypedDict

import numpy as np
from bitbots_utils.utils import get_parameters_from_other_node
from numpy.typing import NDArray

from bitbots_blackboard.capsules import AbstractBlackboardCapsule, cached_capsule_function


class RobotAssignment(TypedDict):
    role: str
    goal_pose: NDArray[np.float64]


class Role(StrEnum):
    STRIKER = "striker"
    GOALIE = "goalie"
    DEFENDER = "defender"
    SUPPORTER = "supporter"


@dataclass
class Field:
    """Field geometry. Populated from the parameter blackboard in __init__."""

    length: float = 9.0  # x extent (own goal at -length/2, opp goal at +length/2)
    width: float = 6.0  # y extent
    goal_width: float = 1.5  # goal mouth
    margin: float = 0.3  # keep field players this far inside the touchlines


@dataclass
class Params:
    """
    A single pure function `compute_formation(ball, field, n_players, params)` maps a
    ball position to positions for every non-... well, *every* role (goalie, defenders,
    supporter, striker), and a small matplotlib GUI to click a ball and watch the result.

    Core idea
    ---------
    Everything keys off two vectors derived from the ball B and our goal centre G:

        to_ball = normalize(B - G)         # axis pointing from our goal out to the ball
        perp    = (-to_ball.y, to_ball.x)  # perpendicular to that axis

    Defenders sit ON the axis (at a controlled depth from goal) and spread ALONG perp.
    As the ball moves the axis rotates, so the same construction continuously morphs
    from a horizontal defensive line (ball far/central) into a goal-line wall beside
    the goalie (ball close & frontal). No special-casing -> automatically smooth.

    A short pairwise-repulsion pass at the end enforces a minimum separation; it is
    also what shoves the defenders sideways into clean flanking slots when they would
    otherwise pile onto the goalie -> the "wall next to the goalie" emerges for free.
    """

    # goalie
    d_g: float = 0.55  # how far the goalie comes out of the goal (dist from goal centre)
    # defenders
    alpha: float = 0.42  # defender depth as a fraction of |ball-goal|  (push-up factor)
    depth_bias: float = 0.0  # extra defender depth: + = further forward, - = further back
    D_min: float = 0.9  # min defender depth from goal (never tuck behind this)
    D_max: float = 3.8  # max defender depth from goal (high-line cap)
    dz: float = 0.45  # keep defenders at least this far ahead of the goalie
    standoff: float = 1.0  # keep defenders at least this far (goal-side) of the ball
    gap: float = 1.1  # lateral spacing between adjacent defenders
    def_side: float = 0.9  # lateral offset for a lone defender (so it's not on the axis)
    # supporter
    f: float = 1.6  # how far in front of the ball (toward opp goal) the supporter sits
    supp_side: float = 1.2  # supporter lateral offset magnitude (auto-leans toward centre)
    supp_max_x: float = 3.0  # supporter never goes past this x (keeps it out of the opp corner)
    # striker
    kick_offset: float = 0.25  # striker stands this far behind the ball (to push it forward)
    post_margin: float = 0.45  # safety margin inside each goal post for a straight shot
    back_dist: float = 0.21  # within this x of the opp goal & not aligned -> play back to our side
    # separation
    min_sep: float = 0.8  # no two robots closer than this
    sep_iters: int = 8
    # kick lane: keep teammates out of the corridor in front of the ball
    kick_clear: float = 0.7  # half-width of the cleared corridor
    kick_range: float = 3.0  # how far in front of the ball the corridor extends
    # opponent free kick / goal kick / penalty: keep all robots outside this radius
    opp_freekick: bool = False
    opp_freekick_clearance: float = 0.8


class PositioningCapsule(AbstractBlackboardCapsule):
    """Decides and communicates the optimal positions, based on the game situation."""

    def __init__(self, node, blackboard):
        super().__init__(node, blackboard)

        parameters = get_parameters_from_other_node(
            self._node,
            "/parameter_blackboard",
            [
                "field.goal.width",
                "field.markings.penalty_area.size.x",
                "field.size.x",
                "field.size.y",
            ],
        )

        self._field = Field(
            length=parameters["field.size.x"],
            width=parameters["field.size.y"],
            goal_width=parameters["field.goal.width"],
        )
        self._params = Params()
        self._inner = InnerPositioningCapsule()
        self._own_locked_role: str | None = None
        self._own_lock_until: float = 0.0
        self._hysteresis_min: float = self._blackboard.config["role_hysteresis"]["min"]
        self._hysteresis_max: float = self._blackboard.config["role_hysteresis"]["max"]

    def set_opp_freekick_active(self, b: bool) -> None:
        self._params.opp_freekick = b

    @cached_capsule_function
    def get_formation_assignment(self) -> dict[int, RobotAssignment]:
        ball_pose = self._blackboard.world_model.get_best_ball_point_stamped()
        ball = np.array([ball_pose.point.x, ball_pose.point.y])
        robot_poses = self._blackboard.team_data.get_robot_poses()
        passive_robot = self._blackboard.team_data.get_index_of_passive_player()
        self._node.get_logger().info(f"Length of robot_poses: {len(robot_poses)}")

        formation = self._inner._compute_formation(ball, self._field, len(robot_poses), self._params)
        new_items = list(formation.items())
        ordered_jerseys = sorted(robot_poses.keys())
        old_poses = [robot_poses[j] for j in ordered_jerseys]
        pairs = self._inner._match_assignment(old_poses, new_items, ball, passive_robot)
        return {ordered_jerseys[old_idx]: {"role": role, "goal_pose": new_pose} for old_idx, new_pose, role in pairs}

    @cached_capsule_function
    def get_own_role(self) -> str:
        """Return our assigned role, locking it in for a random duration on each change.

        Each robot runs this independently with slightly different world state, so
        hysteresis is local: once we switch to a new role we commit to it for
        Uniform(hysteresis_min, hysteresis_max) seconds before accepting another change.
        """
        own_jersey = self._blackboard.gamestate.get_own_id()
        assigned_role = self.get_formation_assignment()[own_jersey]["role"]
        now = self._node.get_clock().now().nanoseconds / 1e9

        if now < self._own_lock_until:
            return self._own_locked_role  # type: ignore[return-value]

        if assigned_role != self._own_locked_role:
            self._own_locked_role = assigned_role
            self._own_lock_until = now + random.uniform(self._hysteresis_min, self._hysteresis_max)

        return assigned_role


class InnerPositioningCapsule:
    """Pure & deterministic formation computation, independent of ROS and the rest of the system."""

    # --------------------------------------------------------------------------- #
    #  Helpers
    # --------------------------------------------------------------------------- #

    @staticmethod
    def _normalize(v: NDArray[np.float64], fallback: NDArray[np.float64] | None = None) -> NDArray[np.float64]:
        if fallback is None:
            fallback = np.array([1.0, 0.0])
        n = np.linalg.norm(v)
        return v / n if n > 1e-9 else fallback.copy()

    @staticmethod
    def _face(frm: NDArray[np.float64], to: NDArray[np.float64], fallback: float = 0.0) -> float:
        """Heading (rad) to look from `frm` toward `to`."""
        d = np.asarray(to) - np.asarray(frm)
        return float(np.arctan2(d[1], d[0])) if np.linalg.norm(d) > 1e-9 else fallback

    @staticmethod
    def _smoothstep(x: float, lo: float, hi: float) -> float:
        """C1 ramp from 0 (x<=lo) to 1 (x>=hi)."""
        if hi <= lo:
            return float(x >= hi)
        t = np.clip((x - lo) / (hi - lo), 0.0, 1.0)
        return float(t * t * (3 - 2 * t))

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Smallest absolute angle between two headings (rad), in [0, pi]."""
        return float(abs((a - b + np.pi) % (2 * np.pi) - np.pi))

    @staticmethod
    def _clamp_field(p: NDArray[np.float64], fld: Field, margin: float | None = None) -> NDArray[np.float64]:
        """Clamp a point to the playable rectangle (continuous / C0)."""
        m = fld.margin if margin is None else margin
        return np.array(
            [
                np.clip(p[0], -fld.length / 2 + m, fld.length / 2 - m),
                np.clip(p[1], -fld.width / 2 + m, fld.width / 2 - m),
            ]
        )

    # --------------------------------------------------------------------------- #
    #  Role allocation
    # --------------------------------------------------------------------------- #

    @staticmethod
    def _allocate_roles(n: int, ball: NDArray[np.float64] | None = None, field: Field | None = None) -> list[str]:
        """Keep-priority as the count drops: striker > goalie > 1st defender > supporter
        > 2nd defender (and any further players are extra defenders).

        Equivalently, removing players one at a time drops: a defender, then the
        supporter, then the other defender, then the goalie.

        Short-handed (n < 5) with the ball in our own defensive third, the supporter
        is reassigned as an extra defender (note: this is a discrete role switch).
        """
        n_def = 0
        has_support = False
        if n >= 3:
            n_def += 1  # first defender
        if n >= 4:
            has_support = True  # supporter
        n_def += max(n - 4, 0)  # everyone past 4 is another defender

        if has_support and n < 5 and ball is not None and field is not None:
            own_third = -field.length / 2 + field.length / 3.0
            if ball[0] < own_third:
                has_support, n_def = False, n_def + 1

        roles = []
        if n >= 1:
            roles.append(Role.STRIKER)
        if n >= 2:
            roles.append(Role.GOALIE)
        roles += [f"{Role.DEFENDER}_{i}" for i in range(n_def)]
        if has_support:
            roles.append(Role.SUPPORTER)
        return roles

    # --------------------------------------------------------------------------- #
    #  Separation / kick-lane clearance
    # --------------------------------------------------------------------------- #

    def _separate(
        self,
        positions: dict[str, NDArray[np.float64]],
        field: Field,
        params: Params,
        ball: NDArray[np.float64] | None = None,
        aim: NDArray[np.float64] | None = None,
    ) -> None:
        """In-place relaxation: pairwise min-sep + clearing the striker's kick lane.

        Only the striker is a fixed anchor (it must reach the ball); everyone else,
        goalie included, gives way around it. In normal play the goalie is far from
        all teammates so it never moves; it only shifts off its line in the degenerate
        case where the ball sits right in the goal mouth. Continuous in the inputs,
        so small ball moves -> small position moves."""
        fixed = {Role.STRIKER}
        names = list(positions.keys())
        sep = params.min_sep
        # model the kick corridor as a row of fixed repulsors along the aim, each with
        # keep-out radius kick_clear: radial pushes are smooth (no side-flip teleport)
        lane_pts, perp = [], None
        if aim is not None and ball is not None:
            perp = np.array([-aim[1], aim[0]])
            step = max(params.kick_clear * 0.7, 0.3)
            lane_pts = [np.asarray(ball) + s * aim for s in np.arange(step, params.kick_range + 1e-9, step)]
        for _ in range(params.sep_iters):
            disp = {n: np.zeros(2) for n in names}
            for i in range(len(names)):
                for j in range(i + 1, len(names)):
                    a, b = names[i], names[j]
                    diff = positions[a] - positions[b]
                    dist = np.linalg.norm(diff)
                    if dist < sep:
                        dirv = self._normalize(diff, np.array([0.0, 1.0])) if dist > 1e-6 else np.array([0.0, 1.0])
                        overlap = sep - dist
                        a_fixed, b_fixed = a in fixed, b in fixed
                        if a_fixed and b_fixed:
                            continue
                        if a_fixed:
                            disp[b] -= overlap * dirv
                        elif b_fixed:
                            disp[a] += overlap * dirv
                        else:
                            disp[a] += 0.5 * overlap * dirv
                            disp[b] -= 0.5 * overlap * dirv
            # push teammates out of the kick corridor in front of the ball
            for n in names:
                if n in fixed:
                    continue
                for lp in lane_pts:
                    diff = positions[n] - lp
                    dist = np.linalg.norm(diff)
                    if dist < params.kick_clear:
                        dirv = self._normalize(diff, perp)  # exactly on the line -> step sideways
                        disp[n] += (params.kick_clear - dist) * dirv
            for n in names:
                if n in fixed:
                    continue
                positions[n] = self._clamp_field(positions[n] + disp[n], field)

    def _clear_ball(
        self, positions: dict[str, NDArray[np.float64]], ball: NDArray[np.float64], radius: float, field: Field
    ) -> None:
        """Push all robots to at least `radius` distance from `ball` (in-place)."""
        ball = np.asarray(ball)
        for name in positions:
            diff = positions[name] - ball
            dist = np.linalg.norm(diff)
            if dist < radius:
                dirv = self._normalize(diff, fallback=np.array([-1.0, 0.0]))
                positions[name] = self._clamp_field(ball + radius * dirv, field)

    # --------------------------------------------------------------------------- #
    #  Assignment
    # --------------------------------------------------------------------------- #

    def _match_assignment(
        self,
        old_poses: list[list[float] | NDArray[np.float64]],
        new_items: list[tuple[str, NDArray[np.float64]]],
        ball: NDArray[np.float64],
        passiv_player: int | None,
        angle_w: float = 0.3,
    ) -> list[tuple[int, NDArray[np.float64], str]]:
        """Assign physical robots (at `old_poses`) to the new target poses.
        The robot closest to the ball always takes the striker target; the rest are
        matched optimally (Hungarian) to minimise total cost = distance + angle_w *
        heading difference. Returns a list of (old_idx, new_pose, new_role) where
        old_idx is the index into `old_poses`.
        `old_poses`: list of [x, y, theta]; `new_items`: list of (role, [x, y, theta]).
        """
        from scipy.optimize import linear_sum_assignment

        old_poses = [np.asarray(p) for p in old_poses]
        ball = np.asarray(ball)
        n = len(old_poses)

        # striker target -> the old robot nearest the ball
        s_j = next(j for j, (r, _) in enumerate(new_items) if r == Role.STRIKER)

        # Kandidaten nach Distanz zum Ball sortiert
        candidates = sorted(range(n), key=lambda i: np.linalg.norm(old_poses[i][:2] - ball))

        # Den passiven Spieler von der Striker-Wahl ausschließen, falls vorhanden
        if passiv_player is not None:
            s_i = next((i for i in candidates if i != passiv_player), candidates[0])
        else:
            s_i = candidates[0]

        pairs = [(s_i, new_items[s_j][1], new_items[s_j][0])]
        rem_i = [i for i in range(n) if i != s_i]
        rem_j = [j for j in range(len(new_items)) if j != s_j]
        if rem_i:
            cost = np.empty((len(rem_i), len(rem_j)))
            for a, i in enumerate(rem_i):
                for b, j in enumerate(rem_j):
                    op, npose = old_poses[i], new_items[j][1]
                    cost[a, b] = np.linalg.norm(op[:2] - npose[:2]) + angle_w * self._angle_diff(op[2], npose[2])
            ri, ci = linear_sum_assignment(cost)
            for a, b in zip(ri, ci):
                i, j = rem_i[a], rem_j[b]
                pairs.append((i, new_items[j][1], new_items[j][0]))
        return pairs

    # --------------------------------------------------------------------------- #
    #  Formation computation
    # --------------------------------------------------------------------------- #

    def _compute_formation(
        self, ball: NDArray[np.float64], field: Field, n_players: int, params: Params
    ) -> dict[str, NDArray[np.float64]]:
        """Map a ball position -> {role: np.array([x, y, yaw])}. Pure & deterministic.

        yaw is in radians, z-up ROS convention (counter-clockwise from +x).
        To build a ROS PoseStamped use `quat_from_yaw(pose[2])` from
        `bitbots_utils.transforms` which returns a geometry_msgs/Quaternion in
        xyzw order (ROS/tf convention). Internally, transforms3d uses wxyz order;
        `bitbots_utils.transforms` handles the conversion.
        """
        b = ball
        goal = np.array([-field.length / 2.0, 0.0])
        opp = np.array([+field.length / 2.0, 0.0])

        d = np.linalg.norm(b - goal)
        to_ball = self._normalize(b - goal)  # our-goal -> ball
        perp = np.array([-to_ball[1], to_ball[0]])

        roles = self._allocate_roles(n_players, b, field)
        if params.opp_freekick and Role.SUPPORTER in roles:
            n_def = sum(1 for r in roles if r.startswith(Role.DEFENDER + "_"))
            roles[roles.index(Role.SUPPORTER)] = f"{Role.DEFENDER}_{n_def}"
        out = {}
        head = {}  # role -> heading (rad); filled lazily, completed after separation
        kick_aim = None  # striker's kick direction; used to clear the kick lane

        
        # --- defenders: anchor on axis at push-up depth, spread along perp ---------- #
        defender_roles = [r for r in roles if r.startswith(Role.DEFENDER + "_")]
        m = len(defender_roles)
        if m > 0:
            depth = np.clip(
                params.alpha * d + params.depth_bias,  # push up + fwd/back bias
                params.D_min,
                params.D_max,
            )
            depth = min(depth, max(d - params.standoff, 0.0))  # stay goal-side of the ball
            depth = max(depth, params.d_g + params.dz)  # stay ahead of the goalie
            anchor = goal + depth * to_ball
            if m == 1:
                # a single defender: shade to the centre side so it doesn't sit on the
                # striker<->goalie line (otherwise all three are collinear)
                side_dir = -1.0 if b[1] >= 0 else 1.0
                offsets = [params.def_side * side_dir]
            else:
                offsets = [(k - (m - 1) / 2.0) * params.gap for k in range(m)]
            for r, off in zip(defender_roles, offsets):
                out[r] = self._clamp_field(anchor + off * perp, field)

        # --- supporter: slightly in front of the ball, kept inside the field -------- #
        if Role.SUPPORTER in roles:
            # always sit to the centre side of the ball; hard swap so it is never
            # directly in front of the striker, even when the ball is on the centre line
            side_dir = -1.0 if b[1] >= 0 else 1.0  # points toward y=0 (default: centre-ward)
            sup = b + np.array([params.f, params.supp_side * side_dir])
            sup[0] = min(sup[0], params.supp_max_x)  # don't drift into the opponent corner
            out[Role.SUPPORTER] = self._clamp_field(sup, field)

        # --- min-separation repulsion + kick-lane clearance ----------------------- #
        self._separate(out, field, params, b, kick_aim)

        # --- freekick: push every robot outside the mandatory clearance radius ---- #
        if params.opp_freekick:
            self._clear_ball(out, b, params.opp_freekick_clearance, field)

        # --- striker: stands behind the ball opposite the smoothly-chosen kick aim --- #
        if Role.STRIKER in roles:
            if params.opp_freekick:
                # park at the clearance boundary on the goal side, facing the ball
                dir_to_goal = self._normalize(goal - b, fallback=np.array([-1.0, 0.0]))
                out[Role.STRIKER] = self._clamp_field(b + params.opp_freekick_clearance * dir_to_goal, field)
                # heading will be computed in the orientation pass (face ball)
            else:
                h = max(field.goal_width / 2 - params.post_margin, 0.0)  # safe half-mouth
                # aim at the goal, target clamped into the safe mouth: this gives a straight
                # (+x) shot whenever the ball is aligned within the posts, angled otherwise
                target = np.array([opp[0], np.clip(b[1], -h, h)])
                aim_goal = self._normalize(target - b)
                # fallback: play back toward our side (field centre)
                aim_back = self._normalize(np.array([0.0, 0.0]) - b, fallback=np.array([-1.0, 0.0]))
                # blend to back-pass only when close to the opp goal AND not aligned
                near_goal = self._smoothstep(b[0], opp[0] - params.back_dist - 1.0, opp[0] - params.back_dist)
                not_aligned = self._smoothstep(abs(b[1]), h, h + 1.0)
                w_back = near_goal * not_aligned
                # rotate the aim around the ball at constant angular rate (shortest path) so
                # the striker swings smoothly rather than whipping when the two aims oppose
                a0 = np.arctan2(aim_goal[1], aim_goal[0])
                a1 = np.arctan2(aim_back[1], aim_back[0])
                da = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi  # shortest signed turn
                ang = a0 + w_back * da
                aim = np.array([np.cos(ang), np.sin(ang)])
                out[Role.STRIKER] = b - params.kick_offset * aim
                head[Role.STRIKER] = ang  # striker faces where it kicks
                kick_aim = aim

        # --- goalie: on the ball->goal axis, hugging the goal, clamped to the mouth -- #
        if Role.GOALIE in roles:
            g = goal + params.d_g * to_ball
            g = np.array(
                [
                    np.clip(g[0], -field.length / 2 + 0.05, -field.length / 2 + params.d_g),
                    np.clip(g[1], -field.goal_width / 2, field.goal_width / 2),
                ]
            )
            out[Role.GOALIE] = g

        # --- orientations (computed from the final positions) --------------------- #
        for role, p in out.items():
            if role in head:
                continue  # striker already set (faces its kick aim)
            if role == Role.SUPPORTER:
                # face the bisector of (toward ball, toward opp goal): "watch both"
                bis = self._normalize(b - p) + self._normalize(opp - p)
                head[role] = self._face(np.zeros(2), bis, fallback=self._face(p, opp))
            else:
                head[role] = self._face(p, b)  # goalie + defenders face the ball

        return {role: np.array([p[0], p[1], head[role]]) for role, p in out.items()}
