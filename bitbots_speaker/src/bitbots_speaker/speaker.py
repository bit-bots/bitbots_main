#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import subprocess
import os

from dynamic_reconfigure.server import Server
#from bitbots_speaker.cfg import speaker_paramsConfig
# todo fix this to make params with dyn_reconf

from bitbots_speaker.msg import Speak


class Speaker(object):
    """ Uses espeak to say all messages from the speak topic
    """

    # todo auch service anbieten, damit blocking zB für Demo sachen oder wenn motor strom ausgeht damit roboter nicht hinfällt

    # todo halt die fresse knopf einbauen

    def __init__(self):
        rospy.init_node('bitbots_speaker', anonymous=False)
        rospy.Subscriber("speak", Speak, self.incoming_text)

        self.speak_enabled = rospy.get_param("/speaker/speak_enabled", True)  # todo dynamic regonfigure
        self.print_say = rospy.get_param("/speaker/print_say", False)  # todo dynamic regonfigure

#todo        self.server = Server(speaker_paramsConfig, self.reconfigure)

        self.low_prio_queue = []
        self.mid_prio_queue = []
        self.high_prio_queue = []

        self.__run_speaker()

    def __run_speaker(self):
        """ Runs continuisly to wait for messages and speak them"""
        # wait for messages. while true doesn't work well in ROS
        while not rospy.is_shutdown():
            # test if espeak is already running
            if not "espeak " in os.popen("ps xa").read():
                if len(self.high_prio_queue) > 0:
                    self.__say(self.high_prio_queue.pop(0))
                elif len(self.mid_prio_queue) > 0:
                    self.__say(self.mid_prio_queue.pop(0))
                elif len(self.low_prio_queue) > 0:
                    self.__say(self.low_prio_queue.pop(0))
            rospy.sleep(0.5)

    def __say(self, text):
        """ Speak this specific text"""
        command = ("espeak", text)
        try:
            process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE)
            try:
                process.communicate()
            finally:
                try:
                    process.terminate()
                except Exception:  # pylint: disable=W0703
                    pass
        except OSError:
            pass

    def incoming_text(self, msg):
        """ Handles incoming msg on speak topic."""

        if not self.speak_enabled:
            # don't accept new messages
            return

        text = msg.text
        rospy.logerr(text)  # todo nur zum testen
        prio = msg.priority
        new = True

        if prio == msg.LOW_PRIORITY:
            for queued_text in self.low_prio_queue:
                if queued_text == text:
                    new = False
                    break
            if new:
                self.low_prio_queue.append(text)
        elif prio == msg.MID_PRIORITY:
            for queued_text in self.mid_prio_queue:
                if queued_text == text:
                    new = False
                    break
            if new:
                self.mid_prio_queue.append(text)
        else:
            for queued_text in self.high_prio_queue:
                if queued_text == text:
                    new = False
                    break
            if new:
                self.high_prio_queue.append(text)

        if self.print_say:
            rospy.loginfo(text)

    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        self.print_say = config["print"]
        self.speak_enabled = config["talk"]
        # Return the new variables.
        return config
