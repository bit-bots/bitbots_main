#!/usr/bin/env python3
from bitbots_test.decorators import error2failure
from bitbots_test.test_case import WebotsTestCase
from transforms3d.euler import euler2quat
import math
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

    @error2failure
    def test_set_ball_position(self):
        # setup
        original_position = self.get_ball_position()

        # execution
        self.set_ball_position(geometry_msgs.msg.Point(x=original_position.x, y=original_position.y + 1, z=original_position.z))

        # verification
        new_position = self.get_ball_position()
        self.assertInRange(new_position.x, (original_position.x - 0.2, original_position.x + 0.2))
        self.assertInRange(new_position.z, (original_position.z - 0.2, original_position.z + 0.2))
        self.assertNotInRange(new_position.y, (original_position.y - 0.2, original_position.y + 0.2))

    def test_assert_robot_standing(self):
        # assert when robot is standing
        self.assertRobotStanding()

        # execution
        fallen_down = euler2quat(0, math.tau / 4, 0)
        self.set_robot_pose(geometry_msgs.msg.Pose(
            position=self.get_robot_pose().position,
            orientation=geometry_msgs.msg.Quaternion(w=fallen_down[0], x=fallen_down[1], y=fallen_down[2], z=fallen_down[3])
        ))

        # verification
        self.assertRaises(AssertionError, self.assertRobotStanding)


if __name__ == "__main__":
    from bitbots_test import run_rostests
    run_rostests(TestSimulationSupervisorControls)
