#!/usr/bin/env python3
"""Debug GUI for InnerPositioningCapsule.

Exercises the exact capsule code without starting the full stack.
Click the field to move the ball; use the sliders to tweak params.
"""

from typing import TypedDict

import numpy as np
from bitbots_blackboard.capsules.positioning_capsule import Field, InnerPositioningCapsule, Params

_inner = InnerPositioningCapsule()


class State(TypedDict):
    ball: tuple[float, float]
    n: int
    robots: list[np.ndarray]


def run_gui():
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    from matplotlib.widgets import CheckButtons, Slider

    fld = Field()
    params = Params()
    # `robots` holds the *persistent* physical robot positions (index = robot identity),
    # updated in place every frame via `_match_assignment` - unlike the formation targets
    # (which are recomputed from scratch each frame), this gives "robot i" a stable
    # identity across ball moves, so marking one passive keeps referring to the same robot.
    state: State = {"ball": (1.0, 0.5), "n": 5, "robots": []}

    colors = {"goalie": "#e6b800", "striker": "#d62728", "supporter": "#2ca02c"}

    fig, ax = plt.subplots(figsize=(9, 9))
    plt.subplots_adjust(left=0.08, right=0.97, top=0.98, bottom=0.50)

    def _ax(b):
        return plt.axes((0.18, b, 0.72, 0.015))

    s_n = Slider(_ax(0.470), "players", 1, 8, valinit=state["n"], valstep=1)
    s_sep = Slider(_ax(0.440), "min_sep", 0.3, 2.0, valinit=params.min_sep)
    s_alpha = Slider(_ax(0.410), "push α", 0.1, 0.8, valinit=params.alpha)
    s_dbias = Slider(_ax(0.380), "def fwd/back", -2.0, 3.0, valinit=params.depth_bias)
    s_dside = Slider(_ax(0.350), "def side", 0.0, 2.0, valinit=params.def_side)
    s_gap = Slider(_ax(0.320), "def gap", 0.5, 2.0, valinit=params.gap)
    s_f = Slider(_ax(0.290), "supp lead", 0.0, 3.0, valinit=params.f)
    s_side = Slider(_ax(0.260), "supp side", 0.0, 2.5, valinit=params.supp_side)
    s_smax = Slider(_ax(0.230), "supp max x", 0.0, 4.2, valinit=params.supp_max_x)
    s_pmarg = Slider(_ax(0.200), "post margin", 0.0, 1.3, valinit=params.post_margin)
    s_back = Slider(_ax(0.170), "back dist", 0.0, 3.0, valinit=params.back_dist)
    s_kclr = Slider(_ax(0.140), "kick clear", 0.0, 1.5, valinit=params.kick_clear)
    s_gout = Slider(_ax(0.110), "goalie out", 0.2, 2.0, valinit=params.d_g)
    check_sp = CheckButtons(plt.axes((0.18, 0.073, 0.20, 0.022)), ["set play"], [False])
    check_supp = CheckButtons(plt.axes((0.42, 0.073, 0.20, 0.022)), ["supporter"], [params.include_supporter])
    s_spcl = Slider(_ax(0.050), "set play clearance", 0.1, 2.0, valinit=params.opp_set_play_clearance)
    # one checkbox per possible robot identity (0..max players - 1); checkboxes beyond the
    # current player count are simply ignored. CheckButtons (unlike TextBox) doesn't hook
    # resize_event, so it doesn't hit the matplotlib bug where TextBox crashes on any
    # window resize (ResizeEvent has no .inaxes, but TextBox._resize assumes it does).
    max_players = 8
    ax_passive = plt.axes((0.915, 0.05, 0.07, 0.40))  # right of the sliders, below the plot
    ax_passive.set_title("passive\nrobot", fontsize=9)
    check_passive = CheckButtons(ax_passive, [str(i) for i in range(max_players)], [False] * max_players)

    def get_passive_indices(n_robots):
        return [i for i, checked in enumerate(check_passive.get_status()) if checked and i < n_robots]

    def draw():
        ax.clear()
        ax.add_patch(
            Rectangle((-fld.length / 2, -fld.width / 2), fld.length, fld.width, fill=False, color="white", lw=2)
        )
        ax.axvline(0, color="white", lw=1)
        ax.add_patch(Circle((0, 0), 0.75, fill=False, color="white", lw=1))
        for sgn in (-1, 1):
            ax.plot(
                [sgn * fld.length / 2] * 2,
                [-fld.goal_width / 2, fld.goal_width / 2],
                color="#4da6ff" if sgn < 0 else "#ff9999",
                lw=6,
            )
        ax.set_facecolor("#2e7d32")

        params.min_sep, params.alpha, params.gap, params.f = s_sep.val, s_alpha.val, s_gap.val, s_f.val
        params.depth_bias, params.supp_side, params.d_g = s_dbias.val, s_side.val, s_gout.val
        params.supp_max_x, params.def_side = s_smax.val, s_dside.val
        params.post_margin, params.back_dist = s_pmarg.val, s_back.val
        params.kick_clear = s_kclr.val
        opp_set_play = check_sp.get_status()[0]
        params.include_supporter = check_supp.get_status()[0]
        params.opp_set_play_clearance = s_spcl.val
        n = int(s_n.val)

        if opp_set_play:
            ax.add_patch(
                Circle(
                    state["ball"],
                    params.opp_set_play_clearance,
                    fill=False,
                    color="yellow",
                    lw=1.5,
                    ls="--",
                    zorder=2,
                    alpha=0.7,
                )
            )

        form = _inner._compute_formation(state["ball"], fld, n, params, opp_set_play=opp_set_play)
        new_items = list(form.items())

        # (re)seed robot identities from scratch whenever the player count changes, since
        # there's no sensible way to carry old identities over a different-sized roster
        if state["robots"] is None or len(state["robots"]) != len(new_items):
            state["robots"] = [pose for _role, pose in new_items]

        passive_idxs = get_passive_indices(len(state["robots"]))
        robots = state["robots"]
        # assign each persistent robot (by index) to a role target for this frame
        assignments = _inner._match_assignment(list(range(len(robots))), robots, new_items, state["ball"], passive_idxs)

        # ball
        ax.add_patch(Circle(state["ball"], 0.12, color="white", ec="black", zorder=5))

        # striker kick lane + aim arrow
        if "striker" in form:
            th = form["striker"][2]
            aim = np.array([np.cos(th), np.sin(th)])
            perp = np.array([-aim[1], aim[0]])
            b = np.asarray(state["ball"])
            far = b + params.kick_range * aim
            corner = perp * params.kick_clear
            lane = np.array([b + corner, far + corner, far - corner, b - corner])
            ax.add_patch(plt.Polygon(lane, closed=True, color="white", alpha=0.10, zorder=1))
            ax.arrow(
                *state["ball"],
                *(1.4 * aim),
                color="white",
                width=0.02,
                head_width=0.18,
                length_includes_head=True,
                zorder=8,
                alpha=0.9,
            )

        # draw each robot at its assigned position, colored by its current role. The robot's
        # identity (its index) and its passive 'x' stay glued to the same marker even when
        # its role - and therefore its color - changes from frame to frame.
        new_robots = list(robots)
        for old_idx, new_pose, role in assignments:
            old_pose = robots[old_idx]
            p, th = new_pose[:2], new_pose[2]
            c = colors.get(role.split("_")[0], "#1f77b4")
            # dashed trail from where this robot was last frame; fixed high-contrast white
            # (role colors like the green defender/supporter vanish against the pitch), with
            # a hollow ring marking the start so the direction of travel is clear
            ax.plot(
                [old_pose[0], p[0]], [old_pose[1], p[1]], ls=(0, (4, 2)), color="white", lw=1.7, alpha=0.95, zorder=3
            )
            ax.add_patch(Circle(tuple(old_pose[:2]), 0.07, fill=False, ec="white", lw=1.2, alpha=0.8, zorder=3))
            ax.add_patch(Circle(tuple(p), params.min_sep / 2, color=c, alpha=0.18, zorder=2))
            ax.add_patch(Circle(tuple(p), 0.16, color=c, ec="black", zorder=6))
            ax.plot(
                [p[0], p[0] + 0.45 * np.cos(th)],
                [p[1], p[1] + 0.45 * np.sin(th)],
                color="black",
                lw=2.5,
                zorder=7,
                solid_capstyle="round",
            )
            short = role.replace("defender_", "D").replace("supporter", "supp")
            ax.annotate(f"{old_idx}:{short}", (p[0], p[1] - 0.30), color="white", fontsize=8, ha="center", zorder=8)
            if old_idx in passive_idxs:
                ax.plot(p[0], p[1], marker="x", color="black", markersize=13, mew=2.5, zorder=9)
            new_robots[old_idx] = new_pose
        state["robots"] = new_robots

        ax.set_xlim(-fld.length / 2 - 0.5, fld.length / 2 + 0.5)
        ax.set_ylim(-fld.width / 2 - 0.5, fld.width / 2 + 0.5)
        ax.set_aspect("equal")
        ax.set_title(
            "click to move the ball  ·  label = robot#:role  ·  dotted trail = movement since last frame  ·  "
            "'x' = passive (never assigned striker)",
            color="black",
        )
        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes is ax and event.xdata is not None:
            state["ball"] = (event.xdata, event.ydata)
            draw()

    for s in (
        s_n,
        s_sep,
        s_alpha,
        s_dbias,
        s_dside,
        s_gap,
        s_f,
        s_side,
        s_smax,
        s_pmarg,
        s_back,
        s_kclr,
        s_gout,
        s_spcl,
    ):
        s.on_changed(lambda _v: draw())
    check_sp.on_clicked(lambda _label: draw())
    check_supp.on_clicked(lambda _label: draw())
    check_passive.on_clicked(lambda _label: draw())
    fig.canvas.mpl_connect("button_press_event", on_click)
    draw()
    plt.show()


def main():
    run_gui()


if __name__ == "__main__":
    main()
