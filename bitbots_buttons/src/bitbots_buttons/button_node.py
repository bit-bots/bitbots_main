#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import time
from bitbots_buttons.msg import Buttons
from humanoid_league_msgs.msg import Speak

from humanoid_league_speaker.speaker import speak
from bitbots_pause.srv import ManualPenalize


class ButtonNode(object):
    """ This node handles pressing of buttons on the robot. It should be used to call services on other nodes,
    as an sort of event driven architecture for the buttons.
    """

    def __init__(self):
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node("ButtonManager", log_level=log_level, anonymous=False)

        # --- Params ---
        self.speaking_active = rospy.get_param("/buttons/speak_active", False)
        self.short_time = rospy.get_param("/buttons/short_time", 2)
        self.manual_penality_mode = rospy.get_param("/buttons/manual_penalty", False)

        # --- Class variables ---
        self.button1 = False
        self.button2 = False
        self.button1_time = 0
        self.button2_time = 0

        # --- Initialize Topics ---
        rospy.Subscriber("/buttons", Buttons, self.button_cb)
        self.speak_publisher = rospy.Publisher('/speak', Speak, queue_size=10)

        rospy.spin()

    def button_cb(self, msg):
        """Callback for msg about pressed buttons."""
        if msg.button1 and not self.button1:
            # button1 was newly pressed
            self.button1 = True
            self.button1_time = rospy.get_time()
        elif msg.button2 and not self.button2:
            # button2 was newly pressed
            self.button2 = True
            self.button2_time = rospy.get_time()
        elif not msg.button1 and self.button1:
            # button1 not pressed anymore
            self.button1 = False
            if rospy.get_time() - self.button1_time < self.short_time:
                self.button1_short()
            else:
                self.button1_long()
            self.button1_time = 0
        elif not msg.button2 and self.button2:
            # button2 not pressed anymore
            self.button2 = False
            if rospy.get_time() - self.button2_time < self.short_time:
                self.button2_short()
            else:
                self.button2_long()
            self.button2_time = 0

    def button1_short(self):
        rospy.logwarn('Button 1 Pressed short')
        speak("Button 1 pressed short", self.speak_publisher, speaking_active=self.speaking_active)

    def button1_long(self):
        rospy.logwarn('Button 1 Pressed long')
        speak("Button 1 pressed long", self.speak_publisher, speaking_active=self.speaking_active)

    def button2_short(self):
        rospy.logwarn('Button 2 Pressed short')
        speak("Button 2 pressed short", self.speak_publisher, speaking_active=self.speaking_active)
        if self.manual_penality_mode:
            # switch penalty state by calling service on motion
            rospy.wait_for_service("manual_penalize")
            manual_penalize_method = rospy.ServiceProxy("manual_penalize", ManualPenalize)
            try:
                response = manual_penalize_method(2)  # argument 3 for switch
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

    def button2_long(self):
        rospy.logwarn('Button 2 Pressed long')
        speak("Button 2 pressed long", self.speak_publisher, speaking_active=self.speaking_active)


if __name__ == "__main__":
    button = ButtonNode()
    rospy.spin()
