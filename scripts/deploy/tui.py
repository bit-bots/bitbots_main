from __future__ import annotations

from deploy.models import DesiredState, RobotStatus, RobotTarget, Step
from deploy.profiles import ProfileStore
from deploy.reconcile import DeploymentSupervisor
from textual import on
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


class RobotCard(Vertical):
    DEFAULT_CSS = """
    RobotCard {
        width: 38;
        min-width: 32;
        height: 100%;
        border: round $primary;
        padding: 0 1;
        margin-right: 1;
    }
    RobotCard:focus-within {
        border: double $accent;
    }
    RobotCard .title {
        text-style: bold;
        color: $accent;
    }
    RobotCard .steps {
        height: auto;
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
        yield Checkbox(
            f"{self.status.target.name} ({self.status.target.robot_name or self.status.target.host})",
            value=False,
            id=f"select-{self.status.target.name}",
            classes="title",
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
        lines = []
        for step in Step:
            report = status.steps[step]
            lines.append(f"{report.state.value[:4].upper():4} {step.value:<14} {report.summary}")
        self.query_one(f"#steps-{status.target.name}", Static).update("\n".join(lines))
        log = self.query_one(f"#log-{status.target.name}", Log)
        log.clear()
        for line in status.logs:
            if visible_at_level(line, minimum_level):
                log.write_line(line)


class DeployApp(App[None]):
    TITLE = "Bit-Bots Deploy Supervisor"
    BINDINGS = [
        ("a", "apply", "Apply"),
        ("r", "retry", "Retry"),
        ("c", "cancel", "Cancel"),
        ("t", "attach", "Attach"),
        ("f", "focus_selected", "Focus"),
        ("u", "show_all", "Show all"),
        ("delete", "remove_selected", "Remove"),
        ("space", "toggle_focused", "Select"),
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
                yield Button("Focus", id="focus-selected")
                yield Button("Show all", id="show-all")
                yield Button("Remove", id="remove-selected", variant="error")
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

    @on(Button.Pressed, "#focus-selected")
    def focus_selected_button(self) -> None:
        self.action_focus_selected()

    @on(Button.Pressed, "#show-all")
    def show_all_button(self) -> None:
        self.action_show_all()

    @on(Button.Pressed, "#remove-selected")
    async def remove_selected_button(self) -> None:
        await self.action_remove_selected()

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

    def action_focus_selected(self) -> None:
        names = self.selected_names()
        if not names:
            self.notify("Select at least one robot to focus", severity="warning")
            return
        self.focused_names = set(names)
        self._apply_card_filter()

    def action_show_all(self) -> None:
        self.focused_names = None
        self._apply_card_filter()

    async def action_remove_selected(self) -> None:
        names = self.selected_names()
        if not names:
            self.notify("Select at least one robot to remove", severity="warning")
            return
        for name in names:
            try:
                card = self.query_one(f"#robot-{name}", RobotCard)
            except NoMatches:
                continue
            await card.remove()
            await self.supervisor.remove_target(name)
        if self.focused_names is not None:
            self.focused_names.difference_update(names)
            if not self.focused_names:
                self.focused_names = None
        self._apply_card_filter()

    def action_toggle_focused(self) -> None:
        focused = self.focused
        if isinstance(focused, Checkbox) and focused.id and focused.id.startswith("select-"):
            focused.value = not focused.value

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


def visible_at_level(line: str, minimum: str) -> bool:
    levels = {"DEBUG": 10, "INFO": 20, "WARN": 30, "WARNING": 30, "ERROR": 40, "FATAL": 50}
    detected = next((name for name in levels if f"[{name}]" in line.upper()), "INFO")
    return levels[detected] >= levels[minimum]


def teamplayer_summary(status: RobotStatus) -> str:
    components = status.teamplayer.get("components", [])
    if not components:
        return "  teamplayer stopped"
    running = sum(component.get("state") == "running" for component in components)
    enabled = sum(bool(component.get("enabled")) for component in components)
    return f"  teamplayer {running}/{enabled}"
