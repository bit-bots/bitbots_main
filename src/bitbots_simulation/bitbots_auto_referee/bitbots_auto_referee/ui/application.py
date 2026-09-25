"""Native read-only Matplotlib application, kept separate from the referee process."""

import json
import math
import time
from datetime import datetime
from textwrap import wrap

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from bitbots_auto_referee.ui.dashboard import DASHBOARD_TOPIC, dashboard_qos


class DashboardSubscriber(Node):
    def __init__(self):
        super().__init__("auto_referee_ui")
        self.snapshot: dict | None = None
        self.received_at: float | None = None
        self.simulation_changed_at: float | None = None
        self.create_subscription(String, DASHBOARD_TOPIC, self._receive, dashboard_qos())

    def _receive(self, message: String) -> None:
        try:
            snapshot = json.loads(message.data)
            format_snapshot(snapshot)
        except (ValueError, TypeError, KeyError, IndexError, AttributeError, OverflowError) as error:
            self.get_logger().warning(f"Ignoring invalid dashboard snapshot: {error}", throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if self.snapshot is None or snapshot["simulation_time_ns"] != self.snapshot["simulation_time_ns"]:
            self.simulation_changed_at = now
        self.snapshot = snapshot
        self.received_at = now


def format_snapshot(snapshot: dict) -> tuple[str, str, str, str, tuple[str, str, str], list[str]]:
    """Format only received state; the application never extrapolates the game clock."""
    state = snapshot["match"]
    home, away = state["teams"]
    if type(state["secs_remaining"]) is not int:
        raise ValueError("secs_remaining must be an integer")
    seconds = max(0, int(state["secs_remaining"]))
    phase = state["state"].removeprefix("STATE_")
    clock = f"{seconds // 60:02d}:{seconds % 60:02d}"

    def team(number):
        return "–" if number is None or number == 255 else f"Team {number}"

    connected = snapshot["robot_connected"]
    connection = "Warte auf Antwort" if connected is None else "Verbunden" if connected else "Keine aktuelle Antwort"
    simulation = snapshot["simulation_time_ns"]
    details = (
        f"Unterbrochen: {'Ja' if state['stopped'] else 'Nein'}\n"
        f"Halbzeit: {'Erste' if state['first_half'] else 'Zweite'}\n"
        f"Vorbereitungszeit: {state['secondary_time']} s",
        f"Anstoß / Fortsetzung: {team(state['kicking_team'])}\n"
        f"Letzter Ballkontakt: {team(snapshot['last_touch_team_id'])}\n"
        f"Roboter: {connection}",
        f"Simulation: {'Noch keine Daten' if simulation is None else f'{simulation / 1e9:.2f} s'}\n"
        f"Standardsituation: {state['set_play'].removeprefix('SET_PLAY_')}\n"
        f"Spielphase: {state['game_phase'].removeprefix('GAME_PHASE_')}",
    )
    events = [
        f"{datetime.fromisoformat(event['time']).astimezone():%H:%M:%S}  {event['message']}"
        for event in snapshot["events"]
    ]
    if not isinstance(snapshot["published_at"], (int, float)) or not math.isfinite(snapshot["published_at"]):
        raise ValueError("published_at must be a timestamp")
    return (
        f"Home · {team(home['team_number'])}\n{home['score']}\n{home['field_player_color']}",
        f"Away · {team(away['team_number'])}\n{away['score']}\n{away['field_player_color']}",
        phase,
        clock,
        details,
        events,
    )


def run_window(node: DashboardSubscriber) -> None:
    import matplotlib

    matplotlib.use("Qt5Agg")
    import matplotlib.pyplot as plt

    with plt.rc_context({"toolbar": "None", "text.usetex": False, "text.parse_math": False}):
        figure = plt.figure(figsize=(12, 8), layout="constrained", facecolor="#f4f6f8")
        figure.canvas.manager.set_window_title("Bit-Bots AutoRef — Nur Anzeige")
        handler_id = getattr(figure.canvas.manager, "key_press_handler_id", None)
        if handler_id is not None:
            figure.canvas.mpl_disconnect(handler_id)
        grid = figure.add_gridspec(3, 3, height_ratios=(1.2, 1, 2))
        figure.suptitle("Bit-Bots AutoRef · Nur Anzeige", fontsize=18, fontweight="bold")

        def panel(position):
            axis = figure.add_subplot(position)
            axis.set_axis_off()
            return axis

        home_axis, center_axis, away_axis = (panel(grid[0, index]) for index in range(3))
        home = home_axis.text(0.5, 0.8, "Home\n–", ha="center", va="top", fontsize=22, linespacing=1.5)
        away = away_axis.text(0.5, 0.8, "Away\n–", ha="center", va="top", fontsize=22, linespacing=1.5)
        phase = center_axis.text(0.5, 0.9, "–", ha="center", fontsize=17, color="#08734f")
        clock = center_axis.text(0.5, 0.4, "–:–", ha="center", fontsize=44, fontweight="bold")
        status = center_axis.text(0.5, 0.07, "Warte auf AutoRef …", ha="center", fontsize=10, color="#805c17")
        details = [panel(grid[1, index]).text(0, 0.85, "", va="top", fontsize=10, linespacing=2) for index in range(3)]
        events_axis = panel(grid[2, :])
        events_axis.set_title("AutoRef-Entscheidungen und Ereignisse · neueste zuerst", loc="left", fontsize=12)
        events = events_axis.text(0, 1, "Noch keine Ereignisse", va="top", fontsize=10, linespacing=1.6, clip_on=True)

        def refresh():
            if not rclpy.ok():
                plt.close(figure)
                return
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except ExternalShutdownException:
                plt.close(figure)
                return
            if node.snapshot is None or node.received_at is None or node.simulation_changed_at is None:
                return
            data = node.snapshot
            home_text, away_text, phase_text, clock_text, detail_text, event_lines = format_snapshot(data)
            home.set_text(home_text)
            away.set_text(away_text)
            phase.set_text(phase_text)
            clock.set_text(clock_text)
            for artist, content in zip(details, detail_text, strict=True):
                artist.set_text(content)
            now = time.monotonic()
            stale = now - node.received_at > 2 or time.time() - data["published_at"] > 2
            advancing = data["simulation_time_ns"] is not None and now - node.simulation_changed_at < 1.5
            if stale:
                status.set_text("Keine aktuellen AutoRef-Daten")
            elif data["match"]["secs_remaining"] <= 0:
                status.set_text("Spielzeit abgelaufen")
            elif not advancing:
                status.set_text("Warte auf Simulationsfortschritt")
            elif phase_text == "PLAYING" and not data["match"]["stopped"]:
                status.set_text("Spieluhr läuft · Live verbunden")
            else:
                status.set_text("Spieluhr angehalten · Live verbunden")
            status.set_color("#a13520" if stale else "#385565")
            width = max(30, int(events_axis.bbox.width / 7.5))
            max_lines = max(1, int(events_axis.bbox.height / 23))
            lines = [line for event in event_lines for line in wrap(event, width=width, subsequent_indent="          ")]
            if len(lines) > max_lines:
                lines = lines[: max(0, max_lines - 1)] + ["… ältere Ereignisse außerhalb der Ansicht"]
            events.set_text("\n".join(lines) if lines else "Noch keine Ereignisse")
            figure.canvas.draw_idle()

        timer = figure.canvas.new_timer(interval=250)
        timer.add_callback(refresh)
        figure.canvas.mpl_connect("close_event", lambda _event: timer.stop())
        timer.start()
        try:
            plt.show()
        finally:
            timer.stop()
            plt.close(figure)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DashboardSubscriber()
        run_window(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
