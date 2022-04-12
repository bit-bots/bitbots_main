#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rclpy
from rcl_interfaces.msg import Parameter, SetParametersResult, ParameterDescriptor
from rclpy.node import Node
import subprocess
import os
import traceback

from humanoid_league_msgs.msg import Audio


def speak(text, publisher, priority=20, speaking_active=True):
    """ Utility method which can be used by other classes to easily publish a message."""
    if speaking_active:
        msg = Audio()
        msg.priority = priority
        msg.text = text
        publisher.publish(msg)


class Speaker:
    """ Uses espeak to say messages from the speak topic.
    """

    # todo make also a service for demo proposes, which is blocking while talking

    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node("speaker")
        self.node.get_logger().info("Starting speaker")

        # --- Class Variables ---
        self.low_prio_queue = []
        self.mid_prio_queue = []
        self.high_prio_queue = []
        # if you want to change standard startup, do it in the config yaml
        self.speak_enabled = None
        self.print_say = None
        self.message_level = None
        self.amplitude = None
        self.node.declare_parameter("print", True)
        self.node.declare_parameter("talk", True)
        self.node.declare_parameter("msg_level", 0)
        self.node.declare_parameter("amplitude", 7)
        self.print_say = self.node.get_parameter("print").get_parameter_value().bool_value
        self.speak_enabled = self.node.get_parameter("talk").get_parameter_value().bool_value
        self.message_level = self.node.get_parameter("msg_level").get_parameter_value().integer_value
        self.amplitude = self.node.get_parameter("amplitude").get_parameter_value().integer_value
        self.node.add_on_set_parameters_callback(self.on_set_parameters)

        self.female_robots = ["donna", "amy"]

        # --- Initialize Topics ---
        self.node.create_subscription(Audio, "speak", self.speak_cb, 10)

        # --- Start loop ---
        self.run_speaker()

    def on_set_parameters(self, parameters: [Parameter]):
        for parameter in parameters:
            if parameter.name == "print":
                self.print_say = parameter.value.bool_value
            elif parameter.name == "talk":
                self.speak_enabled = parameter.value.bool_value
            elif parameter.name == "msg_level":
                self.message_level = parameter.value.int_value
            elif parameter.name == "amplitude":
                self.amplitude = parameter.value.int_value
            else:
                self.node.get_logger().info("Unknown parameter: " + parameter.name)
        return SetParametersResult(successful=True)

    def run_speaker(self):
        """ Runs continuously to wait for messages and speaks them."""
        rate = self.node.create_rate(20)
        while rclpy.ok():
            # test if espeak is already running and speak is enabled
            if not "espeak " in os.popen("ps xa").read() and self.speak_enabled:
                # take the highest priority message first
                if len(self.high_prio_queue) > 0:
                    text, is_file = self.high_prio_queue.pop(0)
                    self.say(text, is_file)
                elif len(self.mid_prio_queue) > 0 and self.message_level <= 1:
                    text, is_file = self.mid_prio_queue.pop(0)
                    self.say(text, is_file)
                elif len(self.low_prio_queue) > 0 and self.message_level == 0:
                    text, is_file = self.low_prio_queue.pop(0)
                    self.say(text)
            # wait a bit to eat not all the performance
            rclpy.spin_once(self.node)
            #rate.sleep()

    def say(self, text, file=False):
        """ Speak this specific text"""
        # todo make volume adjustable, some how like this
        #        command = ("espeak", "-a", self.amplitude, text)
        arguments = []
        if os.getenv('ROBOT_NAME') in self.female_robots:
            arguments.append("-p 99")
        arguments.append(f"-a {self.amplitude}")
        command = ("espeak", *arguments, text)
        try:
            # we start a new process for espeak, so this node can recieve more text while speaking
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                process.communicate()
            finally:
                try:
                    process.terminate()
                except Exception:  # pylint: disable=W0703
                    print(traceback.format_exc())
        except OSError:
            print(traceback.format_exc())

    def speak_cb(self, msg):
        """ Handles incoming msg on speak topic."""
        # first decide if it's a file or a text
        is_file = False
        text = msg.text
        if text is None:
            text = msg.filename
            is_file = True
            if text is None:
                # message has no content at all
                self.node.get_logger().warn("Speaker got message without content.")
                return
        prio = msg.priority
        new = True

        # if printing is enabled and it's a text, print it
        if self.print_say and not is_file:
            self.node.get_logger().info("Said: " + text)

        if not self.speak_enabled:
            # don't accept new messages
            return

        if prio == 0 and self.message_level == 0:
            for queued_text in self.low_prio_queue:
                if queued_text == (text, is_file):
                    new = False
                    break
            if new:
                self.low_prio_queue.append((text, is_file))
        elif prio == 1 and self.message_level <= 1:
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


def main():
    Speaker()


if __name__ == "__main__":
    main()
