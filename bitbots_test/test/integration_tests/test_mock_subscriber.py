#!/usr/bin/env python3
import rospy
import rostest
import time
from std_msgs.msg import String
from bitbots_test.test_case import RosNodeTestCase
from bitbots_test.mocks import MockSubscriber


class MockSubscriberTestCase(RosNodeTestCase):
    sub: MockSubscriber
    pub: rospy.Publisher

    def setUp(self) -> None:
        super().setUp()
        self.pub = rospy.Publisher(self.test_topic, String, queue_size=10)
        self.sub = MockSubscriber(self.test_topic, String, queue_size=10)
        self.sub.wait_until_connected()

    def tearDown(self) -> None:
        super().tearDown()
        self.sub.unregister()
        self.pub.unregister()

    def test_assert_message_received(self):
        # execution
        self.pub.publish(String("this is a test"))

        # verification
        time.sleep(0.2)
        self.sub.assert_message_received()
        self.sub.assert_message_received(String("this is a test"))
        self.assertRaises(AssertionError, lambda: self.sub.assert_message_received(String("something else")))

    def test_assert_messages_received(self):
        # execution
        self.pub.publish(String("test 1"))
        self.pub.publish(String("test 2"))
        self.pub.publish(String("test 3"))

        # verification
        time.sleep(0.2)
        self.sub.assert_messages_received([String("test 1"), String("test 2"), String("test 3")])
        self.sub.assert_messages_received([String("test 2"), String("test 1"), String("test 3")], any_order=True)
        self.assertRaises(AssertionError, lambda: self.sub.assert_messages_received([String("test2"), String("test1")]))

    def test_assert_nothing_received(self):
        # verification
        self.sub.assert_nothing_received()

    def test_assert_one_message_received(self):
        # execution
        self.pub.publish(String("test"))

        # verification
        time.sleep(0.2)
        self.sub.assert_one_message_received()
        self.sub.assert_one_message_received(String("test"))

        self.pub.publish(String("another test"))
        time.sleep(0.2)
        self.assertRaises(AssertionError, lambda: self.sub.assert_one_message_received())

    def test_reset(self):
        # setup
        self.pub.publish(String("test"))
        time.sleep(0.2)
        self.sub.assert_one_message_received()

        # execution
        self.sub.reset()

        # verification
        self.sub.assert_nothing_received()
        self.assertRaises(AssertionError, lambda: self.sub.assert_one_message_received())
        self.assertRaises(AssertionError, lambda: self.sub.assert_message_received())


if __name__ == "__main__":
    rostest.rosrun("bitbots_test", MockSubscriberTestCase.__name__, MockSubscriberTestCase)
