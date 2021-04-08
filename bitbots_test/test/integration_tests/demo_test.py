#!/usr/bin/env python3
import rospy
import rostest
import time
from unittest.mock import Mock
from std_msgs.msg import String
from bitbots_test.test_case import RosNodeTestCase


class DemoTestCase(RosNodeTestCase):
    def test_publish_and_subscribe(self):
        # setup
        mock_callback = Mock()
        sub = rospy.Subscriber("demo", String, callback=mock_callback, queue_size=1)
        pub = rospy.Publisher("demo", String, queue_size=1, latch=True)

        # execution
        pub.publish(String("this is a demo"))

        # verification
        time.sleep(0.2)
        mock_callback.assert_called_once()


if __name__ == "__main__":
    rostest.rosrun("bitbots_test", DemoTestCase.__name__, DemoTestCase)
