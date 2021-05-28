#!/usr/bin/env python3
import rospy
from bitbots_localization.srv import ResetFilter
from bitbots_test.test_case import WebotsTestCase
from bitbots_test.mocks import MockSubscriber
from geometry_msgs.msg import PoseWithCovarianceStamped


class TestInitialLocalizationSide(WebotsTestCase):
    def test_inital_localization_side(self):
        # Reset localization
        rospy.wait_for_service('reset_localization')
        reset_filter_proxy = rospy.ServiceProxy('reset_localization', ResetFilter)
        reset_filter_proxy(0, None, None)

        # Check if we heard from the localization
        self.localization_estimation = None
        sub = MockSubscriber("/pose_with_covariance", PoseWithCovarianceStamped, callback=self.localization_estimation_cb)
        self.with_assertion_grace_period(lambda: sub.assertMessageReceived(), t=5000)

        # Get the position from the simulation
        #self.localization_estimation.pose.pose.position

        # Check if we are localized correctly

        #assert False, f"{self.get_robot_pose()} {self.localization_estimation.pose.pose.position}"

        self.with_assertion_grace_period(lambda: self.assertRobotPosition(self.localization_estimation.pose.pose.position), t=15000)

    def behavior_position_to_webots(self, point):
        return point


    def localization_estimation_cb(self, msg):
        self.localization_estimation = msg


if __name__ == "__main__":
    from bitbots_test import run_rostests
    run_rostests(TestInitialLocalizationSide)
