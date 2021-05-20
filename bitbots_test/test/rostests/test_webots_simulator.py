#!/usr/bin/env python3
from bitbots_test.test_case import WebotsTestCase


class TestSimulationSupervisorControls(WebotsTestCase):
    def test_simulator_running(self):
        # this gets called during setup but this makes the test more explicit
        self.wait_for_simulator()


if __name__ == "__main__":
    from bitbots_test import run_rostests
    run_rostests(TestSimulationSupervisorControls)
