"""
SpeakerCapsule
^^^^^^^^^^^^^^
"""

import rospy
from humanoid_league_msgs.msg import Speak


class SpeakerCapsule:
    def __init__(self):
        self.speaker = None  # type: rospy.Publisher
        self.speak_message = Speak()

    def say(self, message, priority=Speak.LOW_PRIORITY):
        self.speak_message.priority = priority
        self.speak_message.text = message
        self.speaker.publish(self.speak_message)
