#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import subprocess
import os

from dynamic_reconfigure.server import Server
#todo find the problem with this import
from bitbots_speaker.cfg import speaker_paramsConfig

from humanoid_league_msgs.msg import Speak


class Speaker(object):
    """ Uses espeak to say all messages from the speak topic
    """

    # todo make also a service for demo proposes, which is blocking while talking

    def __init__(self):
        rospy.init_node('bitbots_speaker', anonymous=False)
        rospy.Subscriber("speak", Speak, self.incoming_text)

        self.speak_enabled = rospy.get_param("/speaker/speak_enabled", True)
        self.print_say = rospy.get_param("/speaker/print_say", False)
        self.message_level = rospy.get_param("/speaker/message_level", Speak.LOW_PRIORITY)
        self.amplitude = rospy.get_param("/speaker/amplitude", 10)

        self.server = Server(speaker_paramsConfig, self.reconfigure)

        self.low_prio_queue = []
        self.mid_prio_queue = []
        self.high_prio_queue = []

        self.__run_speaker()

    def __run_speaker(self):
        """ Runs continuisly to wait for messages and speak them"""
        # wait for messages. while true doesn't work well in ROS
        while not rospy.is_shutdown():
            print self.message_level
            # test if espeak is already running and speak is enabled
            if not "espeak " in os.popen("ps xa").read() and self.speak_enabled:
                if len(self.high_prio_queue) > 0:
                    text, is_file = self.high_prio_queue.pop(0)
                    self.__say(text, is_file)
                elif len(self.mid_prio_queue) > 0 and self.message_level <= 1:
                    text, is_file = self.mid_prio_queue.pop(0)
                    self.__say(text, is_file)
                elif len(self.low_prio_queue) > 0 and self.message_level == 0:
                    text, is_file = self.low_prio_queue.pop(0)
                    self.__say(text)
            rospy.sleep(0.5)

    def __say(self, text, file=False):
        """ Speak this specific text"""
        # todo make volume adjustable, some how like this
#        command = ("espeak", "-a", self.amplitude, text)
        print "text:" + text
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

        is_file = False
        text = msg.text
        if text is None:
            text = msg.filename
            is_file = True
            if text is None:
                return
        prio = msg.priority
        new = True

        if self.print_say and not is_file:
            rospy.loginfo(text)

        if not self.speak_enabled:
            # don't accept new messages
            return

        if prio == msg.LOW_PRIORITY and self.message_level == 0:
            for queued_text in self.low_prio_queue:
                if queued_text == (text, is_file):
                    new = False
                    break
            if new:
                self.low_prio_queue.append((text, is_file))
        elif prio == msg.MID_PRIORITY and self.message_level <= 1:
            for queued_text in self.mid_prio_queue:
                if queued_text == (text, is_file):
                    new = False
                    break
            if new:
                self.mid_prio_queue.append((text, is_file))
        else:
            for queued_text in self.high_prio_queue:
                if queued_text == (text, is_file):
                    new = False
                    break
            if new:
                self.high_prio_queue.append((text, is_file))

    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        self.print_say = config["print"]
        self.speak_enabled = config["talk"]
        if not self.speak_enabled:
            self.low_prio_queue = []
            self.mid_prio_queue = []
            self.high_prio_queue = []
        message_level = config["msg_level"]
        if self.message_level != message_level:
            # delete old lower prio msgs
            if message_level == 2:
                self.mid_prio_queue = []
                self.low_prio_queue = []
            elif message_level == 1:
                self.low_prio_queue = []
            self.message_level = config["msg_level"]
        self.amplitude = config["amplitude"]
        # Return the new variables.
        return config


#todo integrate reading of files

""""
    def speak_file(self, filename, blocking=False, callback=noop):
        ""
        Ausgabe der Datei filename mittels espeak

        :see: :func:`say`
        ""
        cal = (("espeak", "-m", "-f", filename), callback, random.random())
        self._to_saylog(cal, blocking)


"""