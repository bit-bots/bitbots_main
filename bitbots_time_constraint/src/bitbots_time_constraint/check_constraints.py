#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import rostopic
import rospkg
import yaml
import socket
import os

class AutoSubscriber:
    """
    A class which automatically subscribes to a topic as soon as it becomes available and saves the time when messages come in.
    """

    def __init__(self, topic):
        """
        :param topic: Topic to subscribe to
        :type topic: str
        """
        self.topic = topic
        self.latest_timestamp = None

        self.__subscriber = None        # type: rospy.Subscriber
        self.__subscribe()

    def __subscribe(self, backoff=1.0):
        """
        Try to subscribe to the set topic
        :param backoff: How long to wait until another try
        """
        data_class, _, _ = rostopic.get_topic_class(self.topic)
        if data_class is not None:
            # topic is known
            self.__subscriber = rospy.Subscriber(self.topic, data_class, self.__message_callback, queue_size=1, tcp_nodelay=True)
            rospy.loginfo('Subscribed to topic {}'.format(self.topic))

        else:
            # topic is not yet known
            rospy.loginfo('Topic {} is not yet known. Retrying in {} seconds'.format(self.topic, int(backoff)))
            rospy.Timer(
                rospy.Duration(int(backoff)),
                lambda event: self.__subscribe(backoff * 1.2),
                oneshot=True
            )
    
    def get_timestamp(self):
        return self.latest_timestamp

    def __message_callback(self, data):
        self.latest_timestamp = rospy.Time.now()


class ConstraintChecker:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_time_constraint')

        rospy.init_node("bitbots_time_constraint", anonymous=True)

        self.device_name = socket.gethostname()[:-1]

        try:
            config = rospy.get_param('~' + self.device_name)
        except KeyError:
            rospy.logerr("I am not the device which is selected in the config file.")

        self.constraints = config['constraints']
        self.rate = config['check_rate']

        self._check_min_time(self.rate)

        self.topics = set([element['source_topic'] for element in self.constraints])

        self.topic_auto_subscriber_dict = dict()
        for topic in self.topics:
            self.topic_auto_subscriber_dict[topic] = AutoSubscriber(topic)

        self._main_loop()
    
    def _main_loop(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self._check_constraints()
            r.sleep()

    def _check_min_time(self, freq):
        min_time = 1/freq
        too_small_warn_times = [element['warn_delay'] <= min_time for element in self.constraints]
        too_small_error_times = [element['error_delay'] <= min_time for element in self.constraints]

        if any(too_small_warn_times + too_small_error_times):
            rospy.logwarn("Warn delays too small for check freq.")
    
    def _check_constraints(self):
        for constraint in self.constraints:
            source_topic = constraint['source_topic']
            warn_delay = constraint['warn_delay']
            error_delay = constraint['error_delay']
            now = rospy.Time.now()
            
            source_cb = self.topic_auto_subscriber_dict[source_topic]

            source_timestamp = source_cb.get_timestamp()


            if source_timestamp is not None:
                delay = (now -  source_timestamp).to_sec()
                if delay >= error_delay:
                    rospy.logerr_throttle(5, "Time constraint not satisfied! {} Seconds passed after last {} Message!".format(delay, source_topic))
                elif delay >= warn_delay:
                    rospy.logwarn_throttle(5, "Time constraint not satisfied! {} Seconds passed after last {} Message!".format(delay, source_topic))
            else:
                rospy.logwarn_throttle(5, "Never received {}!".format(source_topic))

if __name__ == "__main__":
    checker = ConstraintChecker()
