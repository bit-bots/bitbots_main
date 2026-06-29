#!/usr/bin/env python3
"""Debug GUI for InnerPositioningCapsule.

Exercises the exact capsule code without starting the full stack.
Click the field to move the ball; use the sliders to tweak params.
"""

import numpy as np

from bitbots_blackboard.capsules.positioning_capsule import Field, InnerPositioningCapsule, Params

_inner = InnerPositioningCapsule()


def run_gui():
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    from matplotlib.widgets import CheckButtons, Slider

    fld = Field()
    params = Params()
    state = {"ball": np.array([1.0, 0.5]), "n": 5, "prev": None}

    colors = {"goalie": "#e6b800", "striker": "#d62728", "supporter": "#2ca02c"}

    fig, ax = plt.subplots(figsize=(9, 9))
    plt.subplots_adjust(left=0.08, right=0.97, top=0.98, bottom=0.50)

    def _ax(b):
        return plt.axes([0.18, b, 0.72, 0.015])

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
    check_fk = CheckButtons(plt.axes([0.18, 0.073, 0.20, 0.022]), ["freekick"], [False])
    s_fkcl = Slider(_ax(0.050), "fk clearance", 0.1, 2.0, valinit=params.freekick_clearance)

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
        params.freekick = check_fk.get_status()[0]
        params.freekick_clearance = s_fkcl.val
        n = int(s_n.val)

        if params.freekick:
            ax.add_patch(Circle(state["ball"], params.freekick_clearance, fill=False, color="yellow", lw=1.5, ls="--", zorder=2, alpha=0.7))

        form = _inner._compute_formation(state["ball"], fld, n, params)
        new_items = list(form.items())

        prev = state["prev"]
        if prev is not None and len(prev) == len(new_items):
            for old_idx, new_pose, role in _inner._match_assignment(prev, new_items, state["ball"]):
                old_pose = prev[old_idx]
                c = colors.get(role.split("_")[0], "#1f77b4")
                ax.plot([old_pose[0], new_pose[0]], [old_pose[1], new_pose[1]], ls=":", color=c, lw=1.5, zorder=3)
                ax.add_patch(Circle(old_pose[:2], 0.10, color=c, alpha=0.55, zorder=4))
        state["prev"] = [pose for _role, pose in new_items]

        ax.add_patch(Circle(state["ball"], 0.12, color="white", ec="black", zorder=5))
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
        for role, pose in form.items():
            p, th = pose[:2], pose[2]
            base = role.split("_")[0]
            c = colors.get(base, "#1f77b4")
            ax.add_patch(Circle(p, params.min_sep / 2, color=c, alpha=0.18, zorder=2))
            ax.add_patch(Circle(p, 0.16, color=c, ec="black", zorder=6))
            ax.plot(
                [p[0], p[0] + 0.45 * np.cos(th)],
                [p[1], p[1] + 0.45 * np.sin(th)],
                color="black",
                lw=2.5,
                zorder=7,
                solid_capstyle="round",
            )
            ax.annotate(
                role.replace("defender_", "D").replace("supporter", "supp"),
                p,
                color="white",
                fontsize=8,
                ha="center",
                va="center",
                zorder=8,
            )

        ax.set_xlim(-fld.length / 2 - 0.5, fld.length / 2 + 0.5)
        ax.set_ylim(-fld.width / 2 - 0.5, fld.width / 2 + 0.5)
        ax.set_aspect("equal")
        ax.set_title("click to move the ball  ·  dots = previous assignment (dotted = who moves where)", color="black")
        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes is ax and event.xdata is not None:
            state["ball"] = np.array([event.xdata, event.ydata])
            draw()

    for s in (s_n, s_sep, s_alpha, s_dbias, s_dside, s_gap, s_f, s_side, s_smax, s_pmarg, s_back, s_kclr, s_gout, s_fkcl):
        s.on_changed(lambda _v: draw())
    check_fk.on_clicked(lambda _label: draw())
    fig.canvas.mpl_connect("button_press_event", on_click)
    draw()
    plt.show()


def main():
    run_gui()


if __name__ == "__main__":
    main()
