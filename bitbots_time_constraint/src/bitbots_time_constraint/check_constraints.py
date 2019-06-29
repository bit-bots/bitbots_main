#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import rostopic
import yaml

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
        self.latest_timestamp = rospy.Timer.now()


class DummySubscriber:
    """
    Returns only the current ros time function
    """

    def __init__(self):
        pass
    
    def get_timestamp(self):
        return rospy.Time.now()

class ConstraintChecker:
    def __init__(self):
        with open("../config.yaml", 'r') as stream:
            try:
                constraint_config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                rospy.roserr("Config not found!!!!")
                rospy.signal_shutdown("")

        rate = constraint_config['check_rate']

        self.r = rospy.Rate(rate)

        self.constraints = constraint_config['constraints']

        self._check_min_time(rate)

        source_topics = set([element['source_topic'] for element in self.constraints])
        destination_topic = set([element['destination_topic'] for element in self.constraints])

        self.topics = source_topics.union(destination_topic).discard("NOW")

        self.topic_auto_subscriber_dict = dict()
        for topic in self.topics:
            self.topic_auto_subscriber_dict[topic] = AutoSubscriber(topic)
        
        self.topic_auto_subscriber_dict["NOW"] = DummySubscriber()

        self._main_loop()
    
    def _main_loop(self):
        while not rospy.is_shutdown():
            self._check_constraints()
            self.r.sleep()

    def _check_min_time(self, freq):
        min_time = 1/freq
        too_small_warn_times = [element['warn_delay'] <= min_time for element in self.constraints]
        too_small_error_times = [element['error_delay'] <= min_time for element in self.constraints]

        if any(too_small_warn_times + too_small_error_times):
            rospy.logwarn("Warn delays too small for check freq.")

    def _check_timestamps(self, source_timestamp, destination_timestamp, now, threshold):
        return not ((source_timestamp is not None) and (now - source_timestamp >= rospy.Duration(threshold)) and not (now >= destination_timestamp) and (destination_timestamp >= source_timestamp))
    
    def _check_constraints(self):
        for constraint in self.constraints:
            source_topic = constraint['source_topic']
            destination_topic = constraint['destination_topic']
            warn_delay = constraint['warn_delay']
            error_delay = constraint['error_delay']

            source_cb = self.topic_auto_subscriber_dict[source_topic]
            destination_cb = self.topic_auto_subscriber_dict[destination_topic]

            source_timestamp = source_cb.get_timestamp()
            destination_timestamp = destination_cb.get_timestamp()

            if not self._check_timestamps(source_timestamp, destination_timestamp, rospy.Time.now(), error_delay):
                rospy.logerr("Time constraint not satisfied! {} is required {} Seconds after {}, but timestamps where {} and {}.".format(destination_topic, warn_delay, source_topic,destination_timestamp.to_sec(), source_timestamp.to_sec()))
            elif not self._check_timestamps(source_timestamp, destination_timestamp, rospy.Time.now(), warn_delay):
                rospy.logwarn("Time constraint not satisfied! {} is required {} Seconds after {}, but timestamps where {} and {}.".format(destination_topic, warn_delay, source_topic,destination_timestamp.to_sec(), source_timestamp.to_sec()))


