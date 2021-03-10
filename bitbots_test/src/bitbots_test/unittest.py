from unittest import TestCase
import rospy


class RosNodeTestCase(TestCase):
    """
    A TestCase class specialized for running tests as a ROS-Node

    In detail it does the following:

    - set up and teardown a ros node automatically
    """

    def setUp(self) -> None:
        super().setUp()
        rospy.init_node(type(self).__name__, anonymous=True)
