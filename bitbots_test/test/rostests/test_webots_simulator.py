#!/usr/bin/env python3
from bitbots_test.decorators import error2failure
from bitbots_test.test_case import WebotsTestCase
import geometry_msgs.msg


class TestSimulationSupervisorControls(WebotsTestCase):
    @error2failure
    def test_wait_for_simulator(self):
        # this gets called during setup but this makes the test more explicit
        self.wait_for_simulator()

    @error2failure
    def test_reset_simulation(self):
        # this gets called during setup bit this makes the test more explicit
        self.reset_simulation()

    @error2failure
    def test_set_robot_pose(self):
        # execution
        original_pose = self.get_robot_pose()
        self.set_robot_pose(geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=original_pose.position.x, y=0, z=original_pose.position.z),
            orientation=original_pose.orientation
        ))

        # verification
        self.assertRaises(AssertionError, lambda: self.assertRobotPosition(original_pose.position))

    @error2failure
    def test_get_robot_pose(self):
        # execution
        pose = self.get_robot_pose()

        # verification
        self.assertIsNotNone(pose)
        self.assertRobotPosition(pose.position)


if __name__ == "__main__":
    from bitbots_test import run_rostests
    run_rostests(TestSimulationSupervisorControls)
