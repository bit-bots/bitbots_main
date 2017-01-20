#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rosparam
import rospy
import subprocess
import os

from dynamic_reconfigure.server import Server
# todo find the problem with this import

from bitbots_speaker.cfg import speaker_paramsConfig

from humanoid_league_msgs.msg import Speak


def speak(text, publisher, priority=Speak.MID_PRIORITY, speaking_active=True):
    if speaking_active:
        msg = Speak()
        msg.priority = priority
        msg.text = text
        publisher.publish(msg)

class Speaker(object):
    """ Uses espeak to say all messages from the speak topic
    """

    # todo make also a service for demo proposes, which is blocking while talking

    def __init__(self):
        # --- Class Variables ---
        self.low_prio_queue = []
        self.mid_prio_queue = []
        self.high_prio_queue = []

        # if you want to change standard startup, do it in the config yaml
        self.speak_enabled = None
        self.print_say = None
        self.message_level = None
        self.amplitude = None

        # --- Initialize Node ---
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node('bitbots_speaker', log_level=log_level, anonymous=False)
        rospy.loginfo("Starting speaker")

        # subscribe to whats to say
        rospy.Subscriber("/speak", Speak, self.incoming_text)

        # dyn reconfigure to turn speaking on/off, set volume and to set priority level
        # will use values from config/speaker_params as start
        self.server = Server(speaker_paramsConfig, self.reconfigure)

        self.__run_speaker()

    def __run_speaker(self):
        """ Runs continuously to wait for messages and speak them"""
        # wait for messages. while true doesn't work well in ROS
        while not rospy.is_shutdown():
            # test if espeak is already running and speak is enabled
            if not "espeak " in os.popen("ps xa").read() and self.speak_enabled:
                # take the highest priority message first
                if len(self.high_prio_queue) > 0:
                    text, is_file = self.high_prio_queue.pop(0)
                    self.__say(text, is_file)
                elif len(self.mid_prio_queue) > 0 and self.message_level <= 1:
                    text, is_file = self.mid_prio_queue.pop(0)
                    self.__say(text, is_file)
                elif len(self.low_prio_queue) > 0 and self.message_level == 0:
                    text, is_file = self.low_prio_queue.pop(0)
                    self.__say(text)
            # wait a bit to eat not all the performance
            rospy.sleep(0.5)

    def __say(self, text, file=False):
        """ Speak this specific text"""
        # todo make volume adjustable, some how like this
        #        command = ("espeak", "-a", self.amplitude, text)
        command = ("espeak", text)
        try:
            # we start a new process for espeak, so this node can recieve more text while speaking
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

        # first decide if it's a file or a text
        is_file = False
        text = msg.text
        if text is None:
            text = msg.filename
            is_file = True
            if text is None:
                # message has no content at all
                rospy.logwarn("Speaker got message without content.")
                return
        prio = msg.priority
        new = True

        # if printing is enabled and it's a text, print it
        if self.print_say and not is_file:
            rospy.loginfo("Said: " + text)

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


# todo integrate reading of files

""""
    def speak_file(self, filename, blocking=False, callback=noop):
        ""
        Ausgabe der Datei filename mittels espeak

        :see: :func:`say`
        ""
        cal = (("espeak", "-m", "-f", filename), callback, random.random())
        self._to_saylog(cal, blocking)


"""

if __name__ == "__main__":
    Speaker()
