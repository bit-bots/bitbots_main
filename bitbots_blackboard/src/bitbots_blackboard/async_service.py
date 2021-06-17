""" https://gist.github.com/jbohren/e33247f7675b5dab05543637098a538b """

import rospy
from concurrent.futures import ThreadPoolExecutor


class AsyncServiceProxy(object):
    """Asynchronous ROS service proxy

    Example 1:

        add_two_ints_async = AsyncServiceProxy('add_two_ints',AddTwoInts)
        fut = add_two_ints_async(1, 2)
        while not fut.done():
            print('Waiting...')
        try:
            print('Result: {}'.format(fut.result()))
        except ServiceException:
            print('Service failed!')

    Example 2:
        def result_cb(fut):
            try:
                print('Result: {}'.format(fut.result()))
            except ServiceException:
                print('Service failed!')

        add_two_ints_async = AsyncServiceProxy('add_two_ints',AddTwoInts,callback=result_cb)
        fut = add_two_ints_async(1, 2)
        while not fut.done():
            print('Waiting...')
    """

    def __init__(self, service_name, service_type, persistent=True,
                 headers=None, callback=None):
        """Create an asynchronous service proxy."""

        self.executor = ThreadPoolExecutor(max_workers=1)
        self.service_proxy = rospy.ServiceProxy(
            service_name,
            service_type,
            persistent,
            headers)
        self.callback = callback

    def __call__(self, *args, **kwargs):
        """Get a Future corresponding to a call of this service."""

        fut = self.executor.submit(self.service_proxy.call, *args, **kwargs)
        if self.callback is not None:
            fut.add_done_callback(self.callback)

        return fut
