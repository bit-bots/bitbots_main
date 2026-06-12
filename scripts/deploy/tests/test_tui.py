import asyncio
from pathlib import Path

from deploy.models import RobotStatus, RobotTarget
from deploy.profiles import ProfileStore
from deploy.tui import DeployApp, RobotCard, visible_at_level


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
            await pilot.click("#focus-selected")
            assert app.query_one("#robot-nuc1", RobotCard).display
            assert not app.query_one("#robot-nuc2", RobotCard).display

            await pilot.click("#show-all")
            assert app.query_one("#robot-nuc1", RobotCard).display
            assert app.query_one("#robot-nuc2", RobotCard).display

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
