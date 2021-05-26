from typing import *
from unittest.mock import Mock, call

import rospy
import time
from rospy.impl.tcpros import DEFAULT_BUFF_SIZE


class MockSubscriber(rospy.Subscriber):
    """
    Class for registering a mock subscriber to a specific topic and asserting that messages of specific
    forms have been received.
    """
    mock: Mock

    def __init__(self, name, data_class, callback=None, callback_args=None, queue_size=None,
                 buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):
        self.mock = Mock(wraps=callback)
        super().__init__(name, data_class, self.mock, callback_args, queue_size, buff_size, tcp_nodelay)

    __init__.__doc__ = rospy.Subscriber.__init__.__doc__

    def assertMessageReceived(self, message: Any = None):
        """
        Assert that at least one message was received.

        If `message` is defined, assert that the provided message was received.
        If not, it is asserted that **any** message was received.
        """
        if message:
            self.mock.assert_any_call(message)
        else:
            self.mock.assert_called()

    def assertOneMessageReceived(self, message: Any = None):
        """
        Assert that exactly one message was received.

        If `message` is defined, the received message must be equal to the provided one.
        Otherwise **any** received message fulfills this.
        """
        if message:
            self.mock.assert_called_once_with(message)
        else:
            self.mock.assert_called_once()

    def assertMessagesReceived(self, messages: Sequence[Any], any_order: bool = False):
        """
        Assert that all message in `messages` have been received.

        :param messages: The list of messages which must have been received.
        :param any_order: If `False` then the messages must have been received sequentially in the given order.
            Additional messages may have been received before or after.

            If `True`, messages may have been received in any order.
        """
        self.mock.assert_has_calls([call(m) for m in messages], any_order=any_order)

    def assertNothingReceived(self):
        """Assert that no messages have been received"""
        self.mock.assert_not_called()

    def reset(self):
        """
        Reset all tracked information as if this was a new object.

        This can be useful where you want to make a series of assertions that reuse the same object.
        """
        self.mock.reset_mock()

    def wait_until_connected(self, num_pubs: int = 1):
        """Block until this subscriber has connected to at least the specified number of publishers"""
        while self.get_num_connections() < num_pubs:
            time.sleep(0.01)
