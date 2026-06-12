import asyncio
from pathlib import Path

from deploy.models import RobotStatus, RobotTarget
from deploy.profiles import ProfileStore
from deploy.tui import DeployApp, visible_at_level


class FakeController:
    def __init__(self) -> None:
        self.target = RobotTarget("nuc1", "127.0.0.1", "amy")
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
    def __init__(self) -> None:
        self.controllers = {"nuc1": FakeController()}
        self.on_status = None
        self.applied = []

    def start(self) -> None:
        return None

    async def close(self) -> None:
        return None

    def apply(self, names, desired) -> None:
        self.applied.append((names, desired))


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
