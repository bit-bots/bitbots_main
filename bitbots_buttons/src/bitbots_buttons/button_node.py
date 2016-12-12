import rospy
import time
from bitbots_cm730.msg import Buttons
from humanoid_league_msgs.msg import Speak

from bitbots_common.src.bitbots_common.util.speaker import speak
from bitbots_motion.srv import ManualPenalize


class button_node(object):
    def __init__(self):
        rospy.init_node("ButtonManager", anonymous=False)
        rospy.Subscriber("/buttons", Buttons, self.update_buttons)
        self.speak_publisher = rospy.Publisher('/Speak', Speak, queue_size=10)
        self.button1 = False
        self.button2 = False
        self.button1_time = 0
        self.button2_time = 0
        self.speaking_active = rospy.get_param("/buttons/speak_active", False)
        self.short_time = rospy.get_param("/buttons/short_time", 2)
        self.manual_penality_mode = rospy.get_param("/button_modes/manual_penalty", False)

    def update_buttons(self, msg):
        if msg.button1 and not self.button1:
            # button1 was newly pressed
            self.button1 = True
            self.button1_time = time.time()
        elif msg.button2 and not self.button2:
            # button2 was newly pressed
            self.button2 = True
            self.button2_time = time.time()
        elif not msg.button1 and self.button1:
            # button1 not pressed anymore
            self.button1 = False
            if time.time() - self.button1_time < self.short_time:
                self.button1_short()
            else:
                self.button1_long()
            self.button1_time = 0
        elif not msg.button2 and self.button2:
            # button2 not pressed anymore
            self.button2 = False
            if time.time() - self.button2_time < self.short_time:
                self.button1_short()
            else:
                self.button1_long()
            self.button2_time = 0

    def button1_short(self):
        speak("Button 1 pressed short", self.speak_publisher, speaking_active=self.speaking_active)

    def button1_long(self):
        speak("Button 1 pressed long", self.speak_publisher, speaking_active=self.speaking_active)

    def button2_short(self):
        speak("Button 2 pressed short", self.speak_publisher, speaking_active=self.speaking_active)
        if self.manual_penality_mode:
            # switch penalty state by calling service on motion
            rospy.wait_for_service("manual_penalize")
            manual_penalize_method = rospy.ServiceProxy("manual_penalize", ManualPenalize)
            try:
                response = manual_penalize_method(3)  # argument 3 for switch
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

    def button2_long(self):
        speak("Button 2 pressed long", self.speak_publisher, speaking_active=self.speaking_active)
