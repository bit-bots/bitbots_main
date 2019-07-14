import cv2
import rospy
import numpy as np
from collections import deque


class RuntimeEvaluator:
    def __init__(self, name="Runtime", queue_size=100):
        # type: (str, int) -> None
        """
        calculates the average time a method (e.g. get_candidates) takes to work on an image
        allows improved evaluation and comparison of different methods
        :param name: name of the evaluator, allow the identification of the printed results
        :param queue_size: amount of measurements used to calculate the average
        """
        # Todo: measure time for different methods at the same time and compare their run times directly
        self.name = name
        self.timer_running = False
        self.start_time = None
        self.stop_time = None
        self.queue_size = queue_size
        self.queue = deque()  # this is used to queue the values of time measurements
        self.last_measurement = None
        self.count = 0  # this variable counts the amount of measurements in the queue
        if self.queue_size < 1:  # failsafe in case of a wrong value for queue_size
            self.queue_size = 1

    def set_image(self, image=None):
        # type: (None) -> None
        """
        resets all variable once the time should be measured for a new picture
        :param image: we don't use this, but every set_image method of other classes has this parameter
        """
        self.timer_running = False
        self.start_time = None
        self.stop_time = None
        self.last_measurement = None

    def start_timer(self):  # -> call this method before the method that should be measured
        # type: () -> None
        """
        starts the timer if the timer isn't running already
        """
        if not self.timer_running:  # failsafe in case start_timer is called multiple times before stop_timer
            self.start_time = cv2.getTickCount()
            self.timer_running = True

    def stop_timer(self):  # -> call this method after the method that should be measured
        # type: () -> None
        """
        stops the timer and calculates the past time since the start of the timer
        adds this measurement of time to the queue
        """
        self.stop_time = cv2.getTickCount()
        self.timer_running = False
        self.last_measurement = (self.stop_time-self.start_time)/cv2.getTickFrequency()
        self._append_queue()
        # with the addition of a new measurement self.count increases by 1 from 0 to queue_size-1 and goes back to 0
        self.count = (self.count + 1) % self.queue_size

    def _append_queue(self):
        # type: () -> None
        """
        adds a new runtime measurement to the queue while considering the max queue_size
        """
        if self.last_measurement is not None:  # failsafe
            q = self.queue
            q.append(self.last_measurement)
            while len(q) > self.queue_size:  # makes sure the queue doesn't exceed the given size
                q.popleft()
            self.queue = q

    def reset_queue(self):
        # type: () -> None
        """
        resets the queue by creating a new empty queue
        """
        self.queue = deque()

    def print_timer(self):
        # type: () -> None
        """
        calculates the average of all measurements in the queue once enough measurements are collected
         and prints the result
        """
        # the results are only printed out after we collected enough measurements:
        rospy.loginfo("Vision runtime evaluator: {} Progress:".format(self.name)+str(self.count+1)+"/"+str(self.queue_size), name='bitbots_vision_runtime_evaluator')
        if self.count == self.queue_size - 1:
            avg = np.array(self.queue).mean()  # calculates the average of our measurements
            rospy.loginfo("Vision runtime evaluator: {} timer: {}".format(self.name, avg), name='bitbots_vision_runtime_evaluator')
