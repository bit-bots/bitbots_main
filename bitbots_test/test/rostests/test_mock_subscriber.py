#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String
from bitbots_test.test_case import RosNodeTestCase
from bitbots_test.mocks import MockSubscriber


class MockSubscriberTestCase(RosNodeTestCase):
    sub: MockSubscriber
    pub: rospy.Publisher

    def setUp(self) -> None:
        super().setUp()
        self.pub = rospy.Publisher(self.topic, String, queue_size=10)
        self.sub = MockSubscriber(self.topic, String, queue_size=10)
        self.sub.wait_until_connected()

    def tearDown(self) -> None:
        super().tearDown()
        self.sub.unregister()
        self.pub.unregister()

    def test_assert_message_received(self):
        # execution
        self.pub.publish(String("this is a test"))

        # verification
        self.with_assertion_grace_period(t=200, f=self.sub.assertMessageReceived)
        self.sub.assertMessageReceived(String("this is a test"))
        self.assertRaises(AssertionError, lambda: self.sub.assertMessageReceived(String("something else")))

    def test_assert_messages_received(self):
        # execution
        self.pub.publish(String("test 1"))
        self.pub.publish(String("test 2"))
        self.pub.publish(String("test 3"))

        # verification
        self.with_assertion_grace_period(t=200, f=self.sub.assertMessageReceived)
        self.sub.assertMessagesReceived([String("test 1"), String("test 2"), String("test 3")])
        self.sub.assertMessagesReceived([String("test 2"), String("test 1"), String("test 3")], any_order=True)
        self.assertRaises(AssertionError, lambda: self.sub.assertMessagesReceived([String("test2"), String("test1")]))

    def test_assert_nothing_received(self):
        # verification
        self.sub.assertNothingReceived()

    def test_assert_one_message_received(self):
        # execution
        self.pub.publish(String("test"))

        # verification
        self.with_assertion_grace_period(t=200, f=self.sub.assertOneMessageReceived)
        self.sub.assertOneMessageReceived()
        self.sub.assertOneMessageReceived(String("test"))

        self.pub.publish(String("another test"))
        time.sleep(0.2)
        self.assertRaises(AssertionError, lambda: self.sub.assertOneMessageReceived())

    def test_reset(self):
        # setup
        self.pub.publish(String("test"))
        self.with_assertion_grace_period(t=200, f=self.sub.assertOneMessageReceived)

        # execution
        self.sub.reset()

        # verification
        self.sub.assertNothingReceived()
        self.assertRaises(AssertionError, lambda: self.sub.assertOneMessageReceived())
        self.assertRaises(AssertionError, lambda: self.sub.assertMessageReceived())


if __name__ == "__main__":
    from bitbots_test import run_rostests

    run_rostests(MockSubscriberTestCase)
