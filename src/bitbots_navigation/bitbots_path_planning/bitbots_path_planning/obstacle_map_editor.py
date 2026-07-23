#!/usr/bin/env python3
"""Interactive editor for the obstacle maps consumed by the ``obstacle_map`` node.

Draws a top-down view of the field (the ``map`` frame, origin at the field
center) and lets you place obstacles of a few *preconfigured* sizes by clicking.
The result is written as the yaml the ``obstacle_map`` node reads.

Run it (needs a display)::

    ros2 run bitbots_path_planning obstacle_map_editor my_map.yaml
    # or standalone:
    python obstacle_map_editor.py my_map.yaml --field labor

Controls:
    left click empty space   place the selected preset
    left click + drag        move an obstacle
    right click / d          delete the obstacle under the cursor / selected one
    1..9                     pick a preset from the palette
    [ ]  or scroll           rotate the selected box (and the placement angle)
    r                        reset the placement angle to 0
    s                        save to the output file
    c                        clear all obstacles
    q                        quit (matplotlib default; save first with s)

The obstacle sizes are fixed: edit ``PRESETS`` below (or pass ``--presets`` a
yaml list) to change the catalogue.
"""

from __future__ import annotations

import argparse
import math
import os
from typing import Any, Optional

import yaml  # type: ignore[import-untyped]

# Fixed catalogue of placeable obstacles. Each entry is one palette slot; sizes
# are in meters. Boxes carry size_x/size_y, cylinders a radius. ``height`` is
# only used for the 3D marker the obstacle_map node draws in RViz.
PRESETS: list[dict] = [
    {"name": "box 0.6x0.4", "type": "box", "size_x": 0.60, "size_y": 0.40, "height": 1.0},
    {"name": "box 0.4x0.35", "type": "box", "size_x": 0.40, "size_y": 0.35, "height": 1.0},
    # Cylinder diameter 0.33 m -> radius 0.165 m.
    {"name": "cyl d0.33", "type": "cylinder", "radius": 0.165, "height": 1.0},
]

_ROTATION_STEP = math.radians(15.0)  # per key press / scroll notch, boxes only.

# Colors.
_FIELD_GREEN = (0.15, 0.5, 0.25)
_OBSTACLE_FILL = (0.9, 0.5, 0.1, 0.6)
_OBSTACLE_EDGE = (0.5, 0.28, 0.02)
_SELECTED_EDGE = (0.85, 0.1, 0.1)
_GHOST = (0.4, 0.4, 0.4)


class ObstacleMapEditor:
    """State and geometry of the editor. GUI wiring lives in :meth:`run`."""

    def __init__(
        self,
        output_path: str,
        frame: str,
        presets: list[dict],
        field: Optional[dict] = None,
        obstacles: Optional[list[dict]] = None,
    ) -> None:
        self.output_path = output_path
        self.frame = frame
        self.presets = presets
        self.field = field
        self.obstacles: list[dict] = obstacles or []

        self.current_preset = 0
        self.placement_yaw = 0.0  # radians; applied to newly placed boxes
        self.selected: Optional[int] = None
        self._drag: Optional[tuple[int, float, float]] = None  # (index, dx, dy)
        self._mouse: Optional[tuple[float, float]] = None

        # Matplotlib handles, created in run() (matplotlib is untyped, so Any).
        self._fig: Any = None
        self._ax: Any = None

    # ------------------------------------------------------------------ #
    # geometry / data (GUI-independent, unit tested)
    # ------------------------------------------------------------------ #
    def place(self, x: float, y: float) -> int:
        """Place the current preset at ``(x, y)`` and return its index."""
        preset = self.presets[self.current_preset]
        obstacle = {"type": preset["type"], "x": float(x), "y": float(y), "height": float(preset.get("height", 1.0))}
        if preset["type"] == "box":
            obstacle["size_x"] = float(preset["size_x"])
            obstacle["size_y"] = float(preset["size_y"])
            obstacle["yaw"] = float(self.placement_yaw)
        else:
            obstacle["radius"] = float(preset["radius"])
        self.obstacles.append(obstacle)
        return len(self.obstacles) - 1

    def hit(self, x: float, y: float) -> Optional[int]:
        """Index of the top-most obstacle containing ``(x, y)``, or None."""
        # Reverse so the most recently drawn (top) obstacle wins.
        for index in range(len(self.obstacles) - 1, -1, -1):
            obstacle = self.obstacles[index]
            if obstacle["type"] == "cylinder":
                if math.hypot(x - obstacle["x"], y - obstacle["y"]) <= obstacle["radius"]:
                    return index
            else:
                # Rotate the query point into the box frame, then bounds-check.
                yaw = obstacle.get("yaw", 0.0)
                cos_yaw, sin_yaw = math.cos(-yaw), math.sin(-yaw)
                dx, dy = x - obstacle["x"], y - obstacle["y"]
                local_x = cos_yaw * dx - sin_yaw * dy
                local_y = sin_yaw * dx + cos_yaw * dy
                if abs(local_x) <= obstacle["size_x"] / 2 and abs(local_y) <= obstacle["size_y"] / 2:
                    return index
        return None

    def delete(self, index: int) -> None:
        del self.obstacles[index]
        if self.selected == index:
            self.selected = None
        elif self.selected is not None and self.selected > index:
            self.selected -= 1

    def rotate_selected(self, delta: float) -> None:
        """Rotate the placement angle and, if a box is selected, that box."""
        self.placement_yaw = _wrap(self.placement_yaw + delta)
        if self.selected is not None and self.obstacles[self.selected]["type"] == "box":
            self.obstacles[self.selected]["yaw"] = _wrap(self.obstacles[self.selected].get("yaw", 0.0) + delta)

    def to_yaml(self) -> str:
        """Serialize the map to the obstacle_map yaml format (one obstacle/line)."""
        lines = [
            "# Obstacle map authored with obstacle_map_editor.",
            f"frame: {self.frame}",
            "obstacles:",
        ]
        for obstacle in self.obstacles:
            rounded = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in obstacle.items()}
            lines.append("  - " + yaml.safe_dump(rounded, default_flow_style=True, sort_keys=False).strip())
        return "\n".join(lines) + "\n"

    def save(self) -> None:
        with open(self.output_path, "w") as f:
            f.write(self.to_yaml())

    # ------------------------------------------------------------------ #
    # loading helpers
    # ------------------------------------------------------------------ #
    @staticmethod
    def load_map(path: str) -> tuple[Optional[str], list[dict]]:
        """Load an existing obstacle map for editing. Returns (frame, obstacles)."""
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("frame"), list(data.get("obstacles", []))

    @staticmethod
    def load_field(path: str) -> Optional[dict]:
        """Read a bitbots_parameter_blackboard field config into a flat dict."""
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        try:
            field = data["parameter_blackboard"]["ros__parameters"]["field"]
        except (KeyError, TypeError):
            return None
        return {
            "size_x": float(field["size"]["x"]),
            "size_y": float(field["size"]["y"]),
            "padding": float(field["size"].get("padding", 1.0)),
            "goal_width": float(field.get("goal", {}).get("width", 0.0)),
            "goal_depth": float(field.get("goal", {}).get("depth", 0.0)),
            "penalty_x": float(field.get("markings", {}).get("penalty_area", {}).get("size", {}).get("x", 0.0)),
        }

    @staticmethod
    def resolve_field(name: str) -> Optional[str]:
        """Locate a field config yaml by name in bitbots_parameter_blackboard."""
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("bitbots_parameter_blackboard")
        except Exception:
            return None
        path = os.path.join(share, "config", "fields", name, "config.yaml")
        return path if os.path.exists(path) else None

    # ------------------------------------------------------------------ #
    # GUI
    # ------------------------------------------------------------------ #
    def run(self) -> None:
        import matplotlib
        import matplotlib.pyplot as plt

        matplotlib.use("TkAgg", force=True)

        self._fig, self._ax = plt.subplots(figsize=(11, 8))
        self._fig.canvas.mpl_connect("button_press_event", self._on_press)
        self._fig.canvas.mpl_connect("button_release_event", self._on_release)
        self._fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self._fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        try:
            self._fig.canvas.manager.set_window_title("Obstacle Map Editor")
        except Exception:
            pass
        self._redraw()
        plt.show()

    def _on_press(self, event) -> None:
        if event.inaxes != self._ax or event.xdata is None:
            return
        x, y = event.xdata, event.ydata
        if event.button == 3:  # right click: delete under cursor
            index = self.hit(x, y)
            if index is not None:
                self.delete(index)
                self._redraw()
            return
        if event.button != 1:
            return
        index = self.hit(x, y)
        if index is None:
            index = self.place(x, y)
        self.selected = index
        obstacle = self.obstacles[index]
        self._drag = (index, x - obstacle["x"], y - obstacle["y"])
        self._redraw()

    def _on_release(self, event) -> None:
        self._drag = None

    def _on_motion(self, event) -> None:
        if event.inaxes != self._ax or event.xdata is None:
            self._mouse = None
            return
        self._mouse = (event.xdata, event.ydata)
        if self._drag is not None:
            index, off_x, off_y = self._drag
            self.obstacles[index]["x"] = event.xdata - off_x
            self.obstacles[index]["y"] = event.ydata - off_y
        self._redraw()

    def _on_key(self, event) -> None:
        key = event.key
        if key in {str(n) for n in range(1, 10)}:
            preset = int(key) - 1
            if preset < len(self.presets):
                self.current_preset = preset
        elif key in ("d", "delete", "backspace"):
            if self.selected is not None:
                self.delete(self.selected)
            elif self._mouse is not None:
                index = self.hit(*self._mouse)
                if index is not None:
                    self.delete(index)
        elif key in ("]", "."):
            self.rotate_selected(_ROTATION_STEP)
        elif key in ("[", ","):
            self.rotate_selected(-_ROTATION_STEP)
        elif key == "r":
            self.placement_yaw = 0.0
        elif key == "c":
            self.obstacles.clear()
            self.selected = None
        elif key == "s":
            self.save()
            print(f"Saved {len(self.obstacles)} obstacle(s) to {self.output_path}")
        self._redraw()

    def _on_scroll(self, event) -> None:
        self.rotate_selected(_ROTATION_STEP if event.button == "up" else -_ROTATION_STEP)
        self._redraw()

    # ------------------------------------------------------------------ #
    # drawing
    # ------------------------------------------------------------------ #
    def _redraw(self) -> None:
        from matplotlib.patches import Circle, Rectangle

        ax = self._ax
        ax.clear()
        self._draw_field()

        for index, obstacle in enumerate(self.obstacles):
            selected = index == self.selected
            edge = _SELECTED_EDGE if selected else _OBSTACLE_EDGE
            lw = 2.5 if selected else 1.5
            if obstacle["type"] == "cylinder":
                ax.add_patch(
                    Circle(
                        (obstacle["x"], obstacle["y"]),
                        obstacle["radius"],
                        facecolor=_OBSTACLE_FILL,
                        edgecolor=edge,
                        lw=lw,
                    )
                )
            else:
                w, h, yaw = obstacle["size_x"], obstacle["size_y"], obstacle.get("yaw", 0.0)
                ax.add_patch(
                    Rectangle(
                        (obstacle["x"] - w / 2, obstacle["y"] - h / 2),
                        w,
                        h,
                        angle=math.degrees(yaw),
                        rotation_point="center",
                        facecolor=_OBSTACLE_FILL,
                        edgecolor=edge,
                        lw=lw,
                    )
                )
                # Heading tick along the box +x, so the yaw is visible.
                ax.plot(
                    [obstacle["x"], obstacle["x"] + math.cos(yaw) * w / 2],
                    [obstacle["y"], obstacle["y"] + math.sin(yaw) * w / 2],
                    color=edge,
                    lw=lw,
                )

        self._draw_ghost()
        self._decorate()
        self._fig.canvas.draw_idle()

    def _draw_ghost(self) -> None:
        from matplotlib.patches import Circle, Rectangle

        if self._mouse is None or self._drag is not None:
            return
        preset = self.presets[self.current_preset]
        x, y = self._mouse
        if preset["type"] == "cylinder":
            self._ax.add_patch(Circle((x, y), preset["radius"], facecolor="none", edgecolor=_GHOST, ls="--", lw=1.2))
        else:
            w, h = preset["size_x"], preset["size_y"]
            self._ax.add_patch(
                Rectangle(
                    (x - w / 2, y - h / 2),
                    w,
                    h,
                    angle=math.degrees(self.placement_yaw),
                    rotation_point="center",
                    facecolor="none",
                    edgecolor=_GHOST,
                    ls="--",
                    lw=1.2,
                )
            )

    def _draw_field(self) -> None:
        from matplotlib.patches import Rectangle

        ax = self._ax
        ax.set_aspect("equal")
        ax.grid(True, ls=":", alpha=0.4)
        ax.axhline(0, color="0.7", lw=0.5)
        ax.axvline(0, color="0.7", lw=0.5)

        if self.field is None:
            ax.set_xlim(-5, 5)
            ax.set_ylim(-4, 4)
        else:
            f = self.field
            hx, hy = f["size_x"] / 2, f["size_y"] / 2
            pad = f["padding"] + 0.5
            ax.set_xlim(-hx - pad, hx + pad)
            ax.set_ylim(-hy - pad, hy + pad)
            # Pitch, boundary, halfway line.
            ax.add_patch(
                Rectangle(
                    (-hx, -hy), f["size_x"], f["size_y"], facecolor=_FIELD_GREEN, alpha=0.12, edgecolor="0.5", lw=1.5
                )
            )
            ax.plot([0, 0], [-hy, hy], color="0.6", lw=1.0)
            # Goals (drawn outward from each goal line).
            if f["goal_width"] > 0:
                gw, gd = f["goal_width"] / 2, f["goal_depth"]
                for sign in (-1, 1):
                    ax.add_patch(
                        Rectangle((sign * hx, -gw), sign * gd, 2 * gw, facecolor="none", edgecolor="0.5", lw=1.2)
                    )
            # Penalty areas (depth known; width approximated as goal width + 2*depth).
            if f["penalty_x"] > 0 and f["goal_width"] > 0:
                pw = f["goal_width"] / 2 + f["penalty_x"]
                for sign in (-1, 1):
                    ax.add_patch(
                        Rectangle(
                            (sign * hx, -pw),
                            -sign * f["penalty_x"],
                            2 * pw,
                            facecolor="none",
                            edgecolor="0.5",
                            lw=0.8,
                            ls="--",
                        )
                    )

        ax.set_xlabel("x (m, field length / robot forward)")
        ax.set_ylabel("y (m, field width / robot left)")

    def _decorate(self) -> None:
        preset = self.presets[self.current_preset]
        palette = "   ".join(
            f"[{i + 1}]{'*' if i == self.current_preset else ' '}{p['name']}" for i, p in enumerate(self.presets)
        )
        self._ax.set_title(
            f"frame '{self.frame}'   |   {len(self.obstacles)} obstacle(s)   |   "
            f"active: {preset['name']}   placement yaw: {math.degrees(self.placement_yaw):.0f} deg",
            fontsize=10,
        )
        self._fig.suptitle(palette, fontsize=9, y=0.98)
        self._fig.text(
            0.5,
            0.01,
            "left-click place/drag  ·  right-click or d delete  ·  1-9 preset  ·  [ ] / scroll rotate  ·  "
            "r reset yaw  ·  c clear  ·  s save",
            ha="center",
            fontsize=8,
            color="0.3",
        )


def _wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _load_presets(path: str) -> list[dict]:
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    presets = data.get("presets", data if isinstance(data, list) else [])
    if not presets:
        raise ValueError(f"No presets found in {path}.")
    return presets


def main() -> None:
    parser = argparse.ArgumentParser(description="Visually place obstacles on the field and save an obstacle map yaml.")
    parser.add_argument(
        "output", nargs="?", default="obstacle_map.yaml", help="Output yaml path (default: obstacle_map.yaml)."
    )
    parser.add_argument("--load", help="Existing obstacle map yaml to open for editing.")
    parser.add_argument("--frame", default="map", help="Frame the obstacles are defined in (default: map).")
    parser.add_argument(
        "--field", default="labor", help="Field name in bitbots_parameter_blackboard to draw as backdrop."
    )
    parser.add_argument("--field-config", help="Explicit field config yaml (overrides --field).")
    parser.add_argument("--no-field", action="store_true", help="Do not draw a field backdrop.")
    parser.add_argument("--presets", help="Yaml file with a custom preset catalogue.")
    args = parser.parse_args()

    presets = _load_presets(args.presets) if args.presets else PRESETS

    frame = args.frame
    obstacles: list[dict] = []
    load_path = args.load or (args.output if os.path.exists(args.output) else None)
    if load_path and os.path.exists(load_path):
        loaded_frame, obstacles = ObstacleMapEditor.load_map(load_path)
        if loaded_frame:
            frame = loaded_frame
        print(f"Loaded {len(obstacles)} obstacle(s) from {load_path}")

    field = None
    if not args.no_field:
        field_config = args.field_config or ObstacleMapEditor.resolve_field(args.field)
        if field_config and os.path.exists(field_config):
            field = ObstacleMapEditor.load_field(field_config)
        if field is None:
            print(f"No field backdrop (could not resolve '{args.field}'); using a plain grid.")

    editor = ObstacleMapEditor(args.output, frame, presets, field=field, obstacles=obstacles)
    editor.run()


if __name__ == "__main__":
    main()
