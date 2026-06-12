import asyncio
from pathlib import Path

from deploy.models import RobotStatus, RobotTarget
from deploy.profiles import ProfileStore
from deploy.tui import DeployApp, RobotCard, visible_at_level
from textual.widgets import Button, Checkbox


class FakeController:
    def __init__(self, name: str = "nuc1") -> None:
        self.target = RobotTarget(name, "127.0.0.1", name)
        self.status = RobotStatus(self.target)
        self.on_status = None

    def retry(self) -> None:
        return None

    def set_log_stream(self, enabled: bool) -> None:
        return None

    def cancel(self) -> None:
        return None

    async def attach(self) -> int:
        return 0


class FakeSupervisor:
    def __init__(self, names: tuple[str, ...] = ("nuc1",)) -> None:
        self.controllers = {name: FakeController(name) for name in names}
        self.on_status = None
        self.applied = []
        self.removed = []

    def start(self) -> None:
        return None

    async def close(self) -> None:
        return None

    def apply(self, names, desired) -> None:
        self.applied.append((names, desired))

    async def remove_target(self, name: str) -> None:
        self.removed.append(name)
        self.controllers.pop(name)


def test_log_level_filtering() -> None:
    assert visible_at_level("[INFO] ready", "INFO")
    assert not visible_at_level("[DEBUG] details", "INFO")
    assert visible_at_level("[ERROR] failed", "WARN")


def test_mouse_selection_and_apply() -> None:
    profiles = ProfileStore.load(Path(__file__).parents[1])
    supervisor = FakeSupervisor()
    app = DeployApp(profiles, supervisor, default_match="lab")

    async def run() -> None:
        async with app.run_test(size=(180, 50)) as pilot:
            await pilot.click("#select-nuc1")
            await pilot.click("#apply")
            assert supervisor.applied
            assert supervisor.applied[0][0] == ["nuc1"]

    asyncio.run(run())


def test_mouse_focus_and_show_all() -> None:
    profiles = ProfileStore.load(Path(__file__).parents[1])
    supervisor = FakeSupervisor(("nuc1", "nuc2"))
    app = DeployApp(profiles, supervisor, default_match="lab")

    async def run() -> None:
        async with app.run_test(size=(180, 50)) as pilot:
            await pilot.click("#select-nuc1")
            await pilot.click("#focus-toggle")
            assert app.query_one("#robot-nuc1", RobotCard).display
            assert not app.query_one("#robot-nuc2", RobotCard).display
            assert str(app.query_one("#focus-toggle", Button).label) == "Show all"

            await pilot.click("#focus-toggle")
            assert app.query_one("#robot-nuc1", RobotCard).display
            assert app.query_one("#robot-nuc2", RobotCard).display
            assert str(app.query_one("#focus-toggle", Button).label) == "Focus"

    asyncio.run(run())


def test_remove_is_enabled_only_with_robot_selection() -> None:
    profiles = ProfileStore.load(Path(__file__).parents[1])
    supervisor = FakeSupervisor()
    app = DeployApp(profiles, supervisor, default_match="lab")

    async def run() -> None:
        async with app.run_test(size=(180, 50)) as pilot:
            remove = app.query_one("#remove-selected", Button)
            assert remove.disabled
            await pilot.click("#select-nuc1")
            assert not remove.disabled
            await pilot.click("#select-nuc1")
            assert remove.disabled

    asyncio.run(run())


def test_space_uses_focused_widget_native_behavior() -> None:
    profiles = ProfileStore.load(Path(__file__).parents[1])
    supervisor = FakeSupervisor()
    app = DeployApp(profiles, supervisor, default_match="lab")

    async def run() -> None:
        async with app.run_test(size=(180, 50)) as pilot:
            checkbox = app.query_one("#select-nuc1", Checkbox)
            checkbox.focus()
            await pilot.press("space")
            assert checkbox.value

    asyncio.run(run())


def test_robot_columns_fill_width_with_seventy_column_minimum() -> None:
    profiles = ProfileStore.load(Path(__file__).parents[1])
    supervisor = FakeSupervisor(("nuc1", "nuc2"))
    app = DeployApp(profiles, supervisor, default_match="lab")

    async def run() -> None:
        async with app.run_test(size=(180, 50)) as pilot:
            first = app.query_one("#robot-nuc1", RobotCard)
            second = app.query_one("#robot-nuc2", RobotCard)
            assert first.size.width >= 70
            assert second.size.width >= 70
            narrow_width = first.size.width
            await pilot.resize_terminal(240, 50)
            assert first.size.width > narrow_width
            assert first.size.width == second.size.width

    asyncio.run(run())


def test_mouse_removes_selected_robot() -> None:
    profiles = ProfileStore.load(Path(__file__).parents[1])
    supervisor = FakeSupervisor(("nuc1", "nuc2"))
    app = DeployApp(profiles, supervisor, default_match="lab")

    async def run() -> None:
        async with app.run_test(size=(180, 50)) as pilot:
            await pilot.click("#select-nuc1")
            await pilot.click("#remove-selected")
            assert supervisor.removed == ["nuc1"]
            assert not app.query("#robot-nuc1")
            assert app.query_one("#robot-nuc2", RobotCard)

    asyncio.run(run())
