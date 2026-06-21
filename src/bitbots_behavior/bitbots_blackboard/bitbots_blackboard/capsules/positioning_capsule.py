from dataclasses import dataclass
from bitbots_utils.utils import get_parameters_from_other_node
import numpy as np

from bitbots_blackboard.capsules import AbstractBlackboardCapsule, cached_capsule_function


@dataclass
class Field:
    """Field geometry. Populated from the parameter blackboard in __init__."""
    length: float = 9.0       # x extent (own goal at -length/2, opp goal at +length/2)
    width: float = 6.0        # y extent
    goal_width: float = 1.5   # goal mouth
    margin: float = 0.3       # keep field players this far inside the touchlines


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
    d_g: float = 0.55         # how far the goalie comes out of the goal (dist from goal centre)
    # defenders
    alpha: float = 0.42       # defender depth as a fraction of |ball-goal|  (push-up factor)
    depth_bias: float = 0.0   # extra defender depth: + = further forward, - = further back
    D_min: float = 0.9        # min defender depth from goal (never tuck behind this)
    D_max: float = 3.8        # max defender depth from goal (high-line cap)
    dz: float = 0.45          # keep defenders at least this far ahead of the goalie
    standoff: float = 1.0     # keep defenders at least this far (goal-side) of the ball
    gap: float = 1.1          # lateral spacing between adjacent defenders
    def_side: float = 0.9     # lateral offset for a lone defender (so it's not on the axis)
    # supporter
    f: float = 1.6            # how far in front of the ball (toward opp goal) the supporter sits
    supp_side: float = 1.2    # supporter lateral offset magnitude (auto-leans toward centre)
    supp_max_x: float = 3.0   # supporter never goes past this x (keeps it out of the opp corner)
    # striker
    kick_offset: float = 0.25 # striker stands this far behind the ball (to push it forward)
    post_margin: float = 0.45 # safety margin inside each goal post for a straight shot
    back_dist: float = 1.0    # within this x of the opp goal & not aligned -> play back to our side
    # separation
    min_sep: float = 0.8      # no two robots closer than this
    sep_iters: int = 8
    # kick lane: keep teammates out of the corridor in front of the ball
    kick_clear: float = 0.7   # half-width of the cleared corridor
    kick_range: float = 3.0   # how far in front of the ball the corridor extends


class PositioningCapsule(AbstractBlackboardCapsule):
    """Decides and communicates the optimal positions, based on the game situation."""

    # --------------------------------------------------------------------------- #
    #  Config
    # --------------------------------------------------------------------------- #

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

    # --------------------------------------------------------------------------- #
    #  Helpers
    # --------------------------------------------------------------------------- #

    @staticmethod
    def _normalize(v, fallback=np.array([1.0, 0.0])):
        n = np.linalg.norm(v)
        return v / n if n > 1e-9 else fallback.copy()

    @staticmethod
    def _face(frm, to, fallback=0.0):
        """Heading (rad) to look from `frm` toward `to`."""
        d = np.asarray(to) - np.asarray(frm)
        return np.arctan2(d[1], d[0]) if np.linalg.norm(d) > 1e-9 else fallback

    @staticmethod
    def _smoothstep(x, lo, hi):
        """C1 ramp from 0 (x<=lo) to 1 (x>=hi)."""
        if hi <= lo:
            return float(x >= hi)
        t = np.clip((x - lo) / (hi - lo), 0.0, 1.0)
        return t * t * (3 - 2 * t)

    @staticmethod
    def _angle_diff(a, b):
        """Smallest absolute angle between two headings (rad), in [0, pi]."""
        return abs((a - b + np.pi) % (2 * np.pi) - np.pi)

    @staticmethod
    def _clamp_field(p, fld, margin=None):
        """Clamp a point to the playable rectangle (continuous / C0)."""
        m = fld.margin if margin is None else margin
        return np.array([
            np.clip(p[0], -fld.length / 2 + m, fld.length / 2 - m),
            np.clip(p[1], -fld.width / 2 + m, fld.width / 2 - m),
        ])

    # --------------------------------------------------------------------------- #
    #  Role allocation
    # --------------------------------------------------------------------------- #

    @staticmethod
    def _allocate_roles(n, ball=None, field=None):
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
            n_def += 1            # first defender
        if n >= 4:
            has_support = True    # supporter
        n_def += max(n - 4, 0)   # everyone past 4 is another defender

        if has_support and n < 5 and ball is not None and field is not None:
            own_third = -field.length / 2 + field.length / 3.0
            if ball[0] < own_third:
                has_support, n_def = False, n_def + 1

        roles = []
        if n >= 1:
            roles.append("striker")
        if n >= 2:
            roles.append("goalie")
        roles += [f"defender_{i}" for i in range(n_def)]
        if has_support:
            roles.append("supporter")
        return roles

    # --------------------------------------------------------------------------- #
    #  Separation / kick-lane clearance
    # --------------------------------------------------------------------------- #

    def _separate(self, positions: dict, field: Field, params: Params, ball=None, aim=None):
        """In-place relaxation: pairwise min-sep + clearing the striker's kick lane.

        Only the striker is a fixed anchor (it must reach the ball); everyone else,
        goalie included, gives way around it. In normal play the goalie is far from
        all teammates so it never moves; it only shifts off its line in the degenerate
        case where the ball sits right in the goal mouth. Continuous in the inputs,
        so small ball moves -> small position moves.

        If `ball`/`aim` are given, teammates inside the corridor in front of the ball
        (along the kick direction) are pushed sideways out of it so they don't block
        the kick; the push tapers in/out along the corridor to stay smooth.
        """
        fixed = {"striker"}
        names = list(positions.keys())
        sep = params.min_sep
        # model the kick corridor as a row of fixed repulsors along the aim, each with
        # keep-out radius kick_clear: radial pushes are smooth (no side-flip teleport)
        lane_pts, perp = [], None
        if aim is not None and ball is not None:
            perp = np.array([-aim[1], aim[0]])
            step = max(params.kick_clear * 0.7, 0.3)
            lane_pts = [np.asarray(ball) + s * aim
                        for s in np.arange(step, params.kick_range + 1e-9, step)]
        for _ in range(params.sep_iters):
            disp = {n: np.zeros(2) for n in names}
            for i in range(len(names)):
                for j in range(i + 1, len(names)):
                    a, b = names[i], names[j]
                    diff = positions[a] - positions[b]
                    dist = np.linalg.norm(diff)
                    if dist < sep:
                        dirv = self._normalize(diff, np.array([0.0, 1.0])) if dist > 1e-6 \
                            else np.array([0.0, 1.0])
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

    # --------------------------------------------------------------------------- #
    #  Assignment
    # --------------------------------------------------------------------------- #

    def _match_assignment(self, old_poses, new_items, ball, angle_w=0.3):
        """Assign physical robots (at `old_poses`) to the new target poses.

        The robot closest to the ball always takes the striker target; the rest are
        matched optimally (Hungarian) to minimise total cost = distance + angle_w *
        heading difference. Returns a list of (old_pose, new_pose, new_role).
        `old_poses`: list of [x, y, theta]; `new_items`: list of (role, [x, y, theta]).
        """
        from scipy.optimize import linear_sum_assignment

        old_poses = [np.asarray(p) for p in old_poses]
        ball = np.asarray(ball)
        n = len(old_poses)

        # striker target -> the old robot nearest the ball
        s_j = next(j for j, (r, _) in enumerate(new_items) if r == "striker")
        s_i = min(range(n), key=lambda i: np.linalg.norm(old_poses[i][:2] - ball))
        pairs = [(old_poses[s_i], new_items[s_j][1], new_items[s_j][0])]

        rem_i = [i for i in range(n) if i != s_i]
        rem_j = [j for j in range(len(new_items)) if j != s_j]
        if rem_i:
            cost = np.empty((len(rem_i), len(rem_j)))
            for a, i in enumerate(rem_i):
                for b, j in enumerate(rem_j):
                    op, npose = old_poses[i], new_items[j][1]
                    cost[a, b] = (np.linalg.norm(op[:2] - npose[:2])
                                + angle_w * self._angle_diff(op[2], npose[2]))
            ri, ci = linear_sum_assignment(cost)
            for a, b in zip(ri, ci):
                i, j = rem_i[a], rem_j[b]
                pairs.append((old_poses[i], new_items[j][1], new_items[j][0]))
        return pairs

    # --------------------------------------------------------------------------- #
    #  Formation computation
    # --------------------------------------------------------------------------- #

    def _compute_formation(self, ball, field: Field, n_players: int, params: Params):
        """Map a ball position -> {role_name: np.array([x, y, theta])}. Pure & deterministic."""
        B = np.asarray(ball, dtype=float)
        G = np.array([-field.length / 2.0, 0.0])
        opp = np.array([+field.length / 2.0, 0.0])

        d = np.linalg.norm(B - G)
        to_ball = self._normalize(B - G)            # our-goal -> ball
        perp = np.array([-to_ball[1], to_ball[0]])

        roles = self._allocate_roles(n_players, B, field)
        out = {}
        head = {}        # role -> heading (rad); filled lazily, completed after separation
        kick_aim = None  # striker's kick direction; used to clear the kick lane

        # --- striker: stands behind the ball opposite the smoothly-chosen kick aim --- #
        if "striker" in roles:
            h = max(field.goal_width / 2 - params.post_margin, 0.0)  # safe half-mouth
            # aim at the goal, target clamped into the safe mouth: this gives a straight
            # (+x) shot whenever the ball is aligned within the posts, angled otherwise
            target = np.array([opp[0], np.clip(B[1], -h, h)])
            aim_goal = self._normalize(target - B)
            # fallback: play back toward our side (field centre)
            aim_back = self._normalize(np.array([0.0, 0.0]) - B, fallback=np.array([-1.0, 0.0]))
            # blend to back-pass only when close to the opp goal AND not aligned
            near_goal = self._smoothstep(B[0], opp[0] - params.back_dist - 1.0, opp[0] - params.back_dist)
            not_aligned = self._smoothstep(abs(B[1]), h, h + 1.0)
            w_back = near_goal * not_aligned
            # rotate the aim around the ball at constant angular rate (shortest path) so
            # the striker swings smoothly rather than whipping when the two aims oppose
            a0 = np.arctan2(aim_goal[1], aim_goal[0])
            a1 = np.arctan2(aim_back[1], aim_back[0])
            da = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi   # shortest signed turn
            ang = a0 + w_back * da
            aim = np.array([np.cos(ang), np.sin(ang)])
            out["striker"] = B - params.kick_offset * aim
            head["striker"] = ang   # striker faces where it kicks
            kick_aim = aim

        # --- goalie: on the ball->goal axis, hugging the goal, clamped to the mouth -- #
        if "goalie" in roles:
            g = G + params.d_g * to_ball
            g = np.array([
                np.clip(g[0], -field.length / 2 + 0.05, -field.length / 2 + params.d_g),
                np.clip(g[1], -field.goal_width / 2, field.goal_width / 2),
            ])
            out["goalie"] = g

        # --- defenders: anchor on axis at push-up depth, spread along perp ---------- #
        defender_roles = [r for r in roles if r.startswith("defender_")]
        m = len(defender_roles)
        if m > 0:
            D = np.clip(params.alpha * d + params.depth_bias,         # push up + fwd/back bias
                        params.D_min, params.D_max)
            D = min(D, max(d - params.standoff, 0.0))                 # stay goal-side of the ball
            D = max(D, params.d_g + params.dz)                        # stay ahead of the goalie
            anchor = G + D * to_ball
            if m == 1:
                # a single defender: shade to the centre side so it doesn't sit on the
                # striker<->goalie line (otherwise all three are collinear)
                side_dir = -1.0 if B[1] >= 0 else 1.0
                offsets = [params.def_side * side_dir]
            else:
                offsets = [(k - (m - 1) / 2.0) * params.gap for k in range(m)]
            for r, off in zip(defender_roles, offsets):
                out[r] = self._clamp_field(anchor + off * perp, field)

        # --- supporter: slightly in front of the ball, kept inside the field -------- #
        if "supporter" in roles:
            # always sit to the centre side of the ball; hard swap so it is never
            # directly in front of the striker, even when the ball is on the centre line
            side_dir = -1.0 if B[1] >= 0 else 1.0   # points toward y=0 (default: centre-ward)
            sup = B + np.array([params.f, params.supp_side * side_dir])
            sup[0] = min(sup[0], params.supp_max_x)  # don't drift into the opponent corner
            out["supporter"] = self._clamp_field(sup, field)

        # --- min-separation repulsion + kick-lane clearance ----------------------- #
        self._separate(out, field, params, B, kick_aim)

        # --- orientations (computed from the final positions) --------------------- #
        for role, p in out.items():
            if role in head:
                continue                       # striker already set (faces its kick aim)
            if role == "supporter":
                # face the bisector of (toward ball, toward opp goal): "watch both"
                bis = self._normalize(B - p) + self._normalize(opp - p)
                head[role] = self._face(np.zeros(2), bis, fallback=self._face(p, opp))
            else:
                head[role] = self._face(p, B)  # goalie + defenders face the ball

        return {role: np.array([p[0], p[1], head[role]]) for role, p in out.items()}

    @cached_capsule_function
    def get_formation_assignment(self) -> dict:
        ball = self._blackboard.world_model.get_best_ball_point_stamped()
        robot_poses = self._blackboard.team_data.get_robot_poses()
        
        formation = self._compute_formation(ball, self._field, len(robot_poses), self._params)
        new_items = list(formation.items())
        ordered_jerseys = sorted(robot_poses.keys())
        old_poses = [robot_poses[j] for j in ordered_jerseys]
        pairs = self._match_assignment(old_poses, new_items, ball)

        result = {}
        for jersey, old_pose in zip(ordered_jerseys, [robot_poses[j] for j in ordered_jerseys]):
            for ap, new_pose, role in pairs:
                if np.allclose(np.asarray(ap), np.asarray(old_pose)):
                    result[jersey] = {"role": role, "goal_pose": np.asarray(new_pose)}
                    break
        return result
