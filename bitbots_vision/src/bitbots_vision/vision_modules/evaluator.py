import cv2
import rospy
import numpy as np
from collections import deque


class RuntimeEvaluator:
    def __init__(self, name="Runtime", queue_size=100):
        # type: (str, int) -> None
        """
        Calculates the average time a method (e.g. get_candidates) takes to work on an image.
        Allows improved evaluation and comparison of different methods.

        :param name: name of the evaluator, allow the identification of the printed results
        :param queue_size: amount of measurements used to calculate the average
        """
        # Todo: measure time for different methods at the same time and compare their run times directly
        self._name = name
        self._timer_running = False
        self._start_time = None
        self._stop_time = None
        self._queue_size = queue_size
        self._queue = deque()  # this is used to queue the values of time measurements
        self._last_measurement = None
        self._count = 0  # this variable counts the amount of measurements in the queue
        if self._queue_size < 1:  # failsafe in case of a wrong value for queue_size
            self._queue_size = 1

    def set_image(self, image=None):
        # type: (None) -> None
        """
        Resets all variable once the time should be measured for a new picture.
        
        :param image: we don't use this, but every set_image method of other classes has this parameter
        """
        self._timer_running = False
        self._start_time = None
        self._stop_time = None
        self._last_measurement = None

    def start_timer(self):  # -> call this method before the method that should be measured
        # type: () -> None
        """
        Starts the timer if the timer isn't running already.
        """
        if not self._timer_running:  # failsafe in case start_timer is called multiple times before stop_timer
            self._start_time = cv2.getTickCount()
            self._timer_running = True

    def stop_timer(self):  # -> call this method after the method that should be measured
        # type: () -> None
        """
        Stops the timer and calculates the past time since the start of the timer.
        Adds this measurement of time to the queue.
        """
        self._stop_time = cv2.getTickCount()
        self._timer_running = False
        self._last_measurement = (self._stop_time - self._start_time) / cv2.getTickFrequency()
        self._append_queue()
        # with the addition of a new measurement self.count increases by 1 from 0 to queue_size-1 and goes back to 0
        self._count = (self._count + 1) % self._queue_size

    def _append_queue(self):
        # type: () -> None
        """
        Adds a new runtime measurement to the queue while considering the max queue_size.
        """
        if self._last_measurement is not None:  # failsafe
            q = self._queue
            q.append(self._last_measurement)
            while len(q) > self._queue_size:  # makes sure the queue doesn't exceed the given size
                q.popleft()
            self._queue = q

    def reset_queue(self):
        # type: () -> None
        """
        Resets the queue by creating a new empty queue.
        """
        self._queue = deque()

    def print_timer(self):
        # type: () -> None
        """
        Calculates the average of all measurements in the queue once enough measurements are collected and prints the result.
        """
        # the results are only printed out after we collected enough measurements:
        rospy.loginfo("Vision runtime evaluator: {} Progress:".format(self._name) + str(self._count + 1) + "/" + str(self._queue_size), logger_name="vision_evaluator")
        if self._count == self._queue_size - 1:
            avg = np.array(self._queue).mean()  # calculates the average of our measurements
            rospy.loginfo("Vision runtime evaluator: {} timer: {}".format(self._name, avg), logger_name="vision_evaluator")
