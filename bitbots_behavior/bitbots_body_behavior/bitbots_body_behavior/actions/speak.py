from bitbots_blackboard.blackboard import BodyBlackboard
from bitbots_tts.tts import speak
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class Speak(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: BodyBlackboard
        self.text = parameters.get("text", None)

    def perform(self, reevaluate=False):
        speak(self.text, self.blackboard.speak_pub, priority=50)
        return self.pop()
