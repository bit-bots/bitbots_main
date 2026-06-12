from __future__ import annotations

from deploy.models import DesiredState, RobotStatus, RobotTarget, Step
from deploy.profiles import ProfileStore
from deploy.reconcile import DeploymentSupervisor
from textual import events, on
from textual.app import App, ComposeResult
from textual.containers import Horizontal, HorizontalScroll, Vertical
from textual.css.query import NoMatches
from textual.message import Message
from textual.widgets import (
    Button,
    Checkbox,
    Footer,
    Header,
    Input,
    Label,
    Log,
    Select,
    SelectionList,
    Static,
    Switch,
)


class StatusChanged(Message):
    def __init__(self, status: RobotStatus) -> None:
        self.status = status
        super().__init__()


class RobotCloseRequested(Message):
    def __init__(self, name: str) -> None:
        self.name = name
        super().__init__()


class RobotCard(Vertical):
    MINIMUM_WIDTH = 70
    RIGHT_MARGIN = 1

    DEFAULT_CSS = """
    RobotCard {
        width: 1fr;
        min-width: 70;
        height: 100%;
        border: round $primary;
        padding: 0 1;
        margin-right: 1;
    }
    RobotCard:focus-within {
        border: double $accent;
    }
    RobotCard.selected {
        border: double $success;
    }
    RobotCard .card-header {
        width: 1fr;
        height: auto;
        align-vertical: middle;
    }
    RobotCard .selector {
        width: 1fr;
        text-style: bold;
        color: $accent;
    }
    RobotCard .close {
        width: 5;
        min-width: 5;
        height: 3;
    }
    RobotCard .steps {
        height: auto;
        margin-bottom: 1;
    }
    RobotCard Log {
        height: 1fr;
        border-top: solid $surface-lighten-2;
    }
    """

    def __init__(self, status: RobotStatus) -> None:
        super().__init__(id=f"robot-{status.target.name}")
        self.status = status

    def compose(self) -> ComposeResult:
        with Horizontal(classes="card-header"):
            yield Checkbox(
                f"{self.status.target.name} ({self.status.target.robot_name or self.status.target.host})",
                value=False,
                id=f"select-{self.status.target.name}",
                classes="selector",
                tooltip="Include this robot in fleet actions",
            )
            yield Button(
                "X",
                id=f"close-{self.status.target.name}",
                classes="close",
                variant="error",
                tooltip="Close this robot column and its SSH connection",
            )
        yield Label("offline", id=f"connection-{self.status.target.name}")
        yield Static("", id=f"steps-{self.status.target.name}", classes="steps")
        yield Log(id=f"log-{self.status.target.name}", highlight=True, auto_scroll=True)

    def update_status(self, status: RobotStatus, minimum_level: str) -> None:
        self.status = status
        connection = self.query_one(f"#connection-{status.target.name}", Label)
        connection.update(
            f"{'connected' if status.connected else 'offline'}"
            f"  generation {status.applied_generation}/{status.generation}"
            f"{'  source stale' if status.stale else ''}"
            f"{teamplayer_summary(status)}"
        )
        self.query_one(f"#steps-{status.target.name}", Static).update(task_timeline(status))
        log = self.query_one(f"#log-{status.target.name}", Log)
        log.clear()
        for line in status.logs:
            if visible_at_level(line, minimum_level):
                log.write_line(line)

    @on(Button.Pressed)
    def close_button(self, event: Button.Pressed) -> None:
        if event.button.id and event.button.id.startswith("close-"):
            event.stop()
            self.post_message(RobotCloseRequested(self.status.target.name))

    @on(Checkbox.Changed)
    def selection_changed(self, event: Checkbox.Changed) -> None:
        if event.checkbox.id == f"select-{self.status.target.name}":
            self.set_class(event.value, "selected")


class DeployApp(App[None]):
    TITLE = "Bit-Bots Deploy Supervisor"
    BINDINGS = [
        ("a", "apply", "Apply"),
        ("r", "retry", "Retry"),
        ("c", "cancel", "Cancel"),
        ("t", "attach", "Attach"),
        ("f", "toggle_focus", "Focus"),
        ("l", "cycle_log_level", "Log level"),
        ("q", "quit", "Quit"),
    ]
    CSS = """
    #controls {
        height: auto;
        padding: 0 1;
        border-bottom: solid $primary;
    }
    #controls > * {
        margin-right: 1;
    }
    #robots {
        height: 1fr;
        padding: 1;
    }
    #components {
        width: 34;
        height: 8;
    }
    #manual-host {
        width: 24;
    }
    """

    def __init__(
        self,
        profiles: ProfileStore,
        supervisor: DeploymentSupervisor,
        default_match: str,
    ) -> None:
        super().__init__()
        self.profiles = profiles
        self.supervisor = supervisor
        self.default_match = default_match
        self.generation = 0
        self.minimum_level = "INFO"
        self.focused_names: set[str] | None = None

    def compose(self) -> ComposeResult:
        yield Header()
        with Vertical():
            with Horizontal(id="controls"):
                yield Button("Focus", id="focus-toggle", disabled=True)
                yield Select(
                    [(name, name) for name in self.profiles.matches],
                    value=self.default_match,
                    id="match",
                    prompt="Match profile",
                )
                yield Select(
                    [(name, name) for name in self.profiles.balls],
                    value=self.profiles.matches[self.default_match].ball,
                    id="ball",
                    prompt="Ball",
                )
                yield SelectionList[str](
                    ("Low-level hardware", "lowlevel", True),
                    ("Motion", "motion", True),
                    ("Game controller", "game_controller", True),
                    ("Vision", "vision", True),
                    ("IPM", "ipm", True),
                    ("Localization", "localization", True),
                    ("Path planning", "path_planning", True),
                    ("Behavior", "behavior", True),
                    ("Team communication", "teamcom", True),
                    ("World model", "world_model", True),
                    ("Whistle detector", "whistle_detector", True),
                    ("Audio", "audio", True),
                    ("Text to speech", "tts", False),
                    ("Monitoring", "monitoring", False),
                    ("Recording", "record", False),
                    id="components",
                )
                yield Label("Sync")
                yield Switch(value=True, id="sync")
                yield Label("Build")
                yield Switch(value=True, id="build")
                yield Label("Logs")
                yield Switch(value=True, id="show-logs")
                yield Select(
                    [("INFO", "INFO"), ("DEBUG", "DEBUG"), ("WARN", "WARN"), ("ERROR", "ERROR")],
                    value="INFO",
                    id="log-level",
                )
                yield Button("Apply", id="apply", variant="success")
                yield Button("Retry", id="retry", variant="warning")
                yield Button("Cancel", id="cancel", variant="error")
                yield Button("Attach", id="attach", variant="primary")
                yield Input(placeholder="user@host or host", id="manual-host")
                yield Button("Connect", id="connect")
            with HorizontalScroll(id="robots"):
                for controller in self.supervisor.controllers.values():
                    yield RobotCard(controller.status)
        yield Footer()

    def on_mount(self) -> None:
        self.supervisor.on_status = self._status_callback
        for controller in self.supervisor.controllers.values():
            controller.on_status = self._status_callback
            controller.set_log_stream(True)
        self.supervisor.start()
        self.call_after_refresh(self._update_card_widths)

    async def on_unmount(self) -> None:
        await self.supervisor.close()

    def _status_callback(self, status: RobotStatus) -> None:
        self.post_message(StatusChanged(status))

    @on(StatusChanged)
    def show_status(self, message: StatusChanged) -> None:
        try:
            card = self.query_one(f"#robot-{message.status.target.name}", RobotCard)
        except NoMatches:
            return
        card.update_status(message.status, self.minimum_level)

    @on(Select.Changed, "#match")
    def match_changed(self, event: Select.Changed) -> None:
        if event.value is Select.BLANK:
            return
        match = self.profiles.matches[str(event.value)]
        self.query_one("#ball", Select).value = match.ball

    @on(Select.Changed, "#log-level")
    def log_level_changed(self, event: Select.Changed) -> None:
        if event.value is Select.BLANK:
            return
        self.minimum_level = str(event.value)
        for controller in self.supervisor.controllers.values():
            card = self.query_one(f"#robot-{controller.target.name}", RobotCard)
            card.update_status(controller.status, self.minimum_level)

    @on(Switch.Changed, "#show-logs")
    def show_logs_changed(self, event: Switch.Changed) -> None:
        for controller in self.supervisor.controllers.values():
            controller.set_log_stream(event.value)
        for log in self.query("RobotCard Log"):
            log.display = event.value

    @on(Button.Pressed, "#apply")
    async def apply_button(self) -> None:
        await self.action_apply()

    @on(Button.Pressed, "#retry")
    async def retry_button(self) -> None:
        await self.action_retry()

    @on(Button.Pressed, "#attach")
    async def attach_button(self) -> None:
        await self.action_attach()

    @on(Button.Pressed, "#cancel")
    async def cancel_button(self) -> None:
        await self.action_cancel()

    @on(Button.Pressed, "#focus-toggle")
    def focus_toggle_button(self) -> None:
        self.action_toggle_focus()

    @on(RobotCloseRequested)
    async def close_robot(self, event: RobotCloseRequested) -> None:
        await self._remove_robot(event.name)

    @on(Checkbox.Changed)
    def robot_selection_changed(self, event: Checkbox.Changed) -> None:
        if event.checkbox.id and event.checkbox.id.startswith("select-"):
            self._update_fleet_controls()

    @on(Button.Pressed, "#connect")
    async def connect_button(self) -> None:
        value = self.query_one("#manual-host", Input).value.strip()
        if not value:
            return
        user, separator, host = value.partition("@")
        if not separator:
            host, user = user, "nvidia"
        name = host.replace(".", "-")
        controller = self.supervisor.add_target(RobotTarget(name=name, host=host, user=user))
        await self.query_one("#robots", HorizontalScroll).mount(RobotCard(controller.status))
        controller.set_log_stream(self.query_one("#show-logs", Switch).value)
        if self.focused_names is not None:
            self.focused_names.add(name)
            self._apply_card_filter()
        self._update_fleet_controls()
        self.call_after_refresh(self._update_card_widths)
        self.query_one("#manual-host", Input).value = ""

    async def action_apply(self) -> None:
        names = self.selected_names()
        if not names:
            self.notify("Select at least one robot", severity="warning")
            return
        self.generation += 1
        match_name = str(self.query_one("#match", Select).value)
        ball_name = str(self.query_one("#ball", Select).value)
        desired = DesiredState(
            generation=self.generation,
            match=self.profiles.matches[match_name],
            ball=self.profiles.balls[ball_name],
            components=frozenset(self.query_one("#components", SelectionList).selected),
            sync_source=self.query_one("#sync", Switch).value,
            build=self.query_one("#build", Switch).value,
        )
        self.supervisor.apply(names, desired)

    async def action_retry(self) -> None:
        for name in self.selected_names():
            self.supervisor.controllers[name].retry()

    async def action_cancel(self) -> None:
        for name in self.selected_names():
            self.supervisor.controllers[name].cancel()

    async def action_attach(self) -> None:
        names = self.selected_names()
        if len(names) != 1:
            self.notify("Select exactly one robot to attach", severity="warning")
            return
        with self.suspend():
            await self.supervisor.controllers[names[0]].attach()

    def action_toggle_focus(self) -> None:
        if self.focused_names is None:
            names = self.selected_names()
            if not names:
                self.notify("Select at least one robot to focus", severity="warning")
                return
            self.focused_names = set(names)
        else:
            self.focused_names = None
        self._apply_card_filter()
        self._update_fleet_controls()

    async def _remove_robot(self, name: str) -> None:
        try:
            card = self.query_one(f"#robot-{name}", RobotCard)
        except NoMatches:
            return
        await card.remove()
        await self.supervisor.remove_target(name)
        if self.focused_names is not None:
            self.focused_names.discard(name)
            if not self.focused_names:
                self.focused_names = None
        self._apply_card_filter()
        self._update_fleet_controls()

    def action_cycle_log_level(self) -> None:
        levels = ["INFO", "DEBUG", "WARN", "ERROR"]
        level = levels[(levels.index(self.minimum_level) + 1) % len(levels)]
        self.query_one("#log-level", Select).value = level

    def selected_names(self) -> list[str]:
        selected = []
        for name in self.supervisor.controllers:
            checkbox = self.query_one(f"#select-{name}", Checkbox)
            if checkbox.value:
                selected.append(name)
        return selected

    def _apply_card_filter(self) -> None:
        for name in self.supervisor.controllers:
            try:
                card = self.query_one(f"#robot-{name}", RobotCard)
            except NoMatches:
                continue
            card.display = self.focused_names is None or name in self.focused_names
        self._update_card_widths()

    def _update_fleet_controls(self) -> None:
        selected = bool(self.selected_names())
        focus = self.query_one("#focus-toggle", Button)
        focus.label = "Show all" if self.focused_names is not None else "Focus"
        focus.disabled = self.focused_names is None and not selected

    def on_resize(self, event: events.Resize) -> None:
        self._update_card_widths(event.size.width - 2)

    def _update_card_widths(self, available_width: int | None = None) -> None:
        cards = [card for card in self.query(RobotCard) if card.display]
        if not cards:
            return
        if available_width is None:
            available_width = self.query_one("#robots", HorizontalScroll).size.width
        required_width = len(cards) * (RobotCard.MINIMUM_WIDTH + RobotCard.RIGHT_MARGIN)
        width = RobotCard.MINIMUM_WIDTH if required_width > available_width else "1fr"
        for card in cards:
            card.styles.width = width


def visible_at_level(line: str, minimum: str) -> bool:
    levels = {"DEBUG": 10, "INFO": 20, "WARN": 30, "WARNING": 30, "ERROR": 40, "FATAL": 50}
    detected = next((name for name in levels if f"[{name}]" in line.upper()), "INFO")
    return levels[detected] >= levels[minimum]


def task_timeline(status: RobotStatus) -> str:
    reports = [(step, status.steps[step]) for step in Step if status.steps[step].state.value != "pending"]
    if not reports:
        return "No deployment requested"
    lines = []
    for step, report in reports:
        state = report.state.value.upper()
        summary = f" - {report.summary}" if report.summary else ""
        lines.append(f"{state:<9} {step.value}{summary}")
    pending = sum(report.state.value == "pending" for report in status.steps.values())
    if pending:
        lines.append(f"{pending} step{'s' if pending != 1 else ''} queued")
    return "\n".join(lines)


def teamplayer_summary(status: RobotStatus) -> str:
    components = status.teamplayer.get("components", [])
    if not components:
        return "  teamplayer stopped"
    running = sum(component.get("state") == "running" for component in components)
    enabled = sum(bool(component.get("enabled")) for component in components)
    return f"  teamplayer {running}/{enabled}"
