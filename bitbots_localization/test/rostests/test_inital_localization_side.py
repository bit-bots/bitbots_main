#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bitbots_test.test_case import WebotsTestCase
from bitbots_test.mocks import MockSubscriber
from geometry_msgs.msg import PoseWithCovarianceStamped


class TestInitialLocalizationSide(WebotsTestCase):
    def test_inital_localization_side(self):
        # Check if we heard from the localization
        sub = MockSubscriber(
            "/pose_with_covariance",
            PoseWithCovarianceStamped,
            callback=lambda msg: setattr(self, 'localization_estimation', msg))
        self.with_assertion_grace_period(lambda: sub.assertMessageReceived(), t=5000)
        self.with_assertion_grace_period(lambda: self.assertRobotPosition(self.localization_estimation.pose.pose.position), t=15000)


if __name__ == "__main__":
    from bitbots_test import run_rostests
    run_rostests(TestInitialLocalizationSide)
