import os

from ament_index_python import get_package_share_directory
from bitbots_blackboard.blackboard import BodyBlackboard
from bitbots_tts.tts import speak
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class Speak(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: BodyBlackboard
        self.text = parameters.get("text", None).replace("_", " ")

    def perform(self, reevaluate=False):
        speak(self.text, self.blackboard.misc.speak_pub, priority=50)
        return self.pop()


class PlaySound(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.audio_file = os.path.join(
            get_package_share_directory("bitbots_body_behavior"), "config", parameters.get("file")
        )

    def perform(self, reevaluate=False):
        try:
            import playsound
        except ImportError:
            return self.pop()

        try:
            playsound.playsound(self.audio_file, block=False)
        except playsound.PlaysoundException:
            pass

        return self.pop()
