import random

from bitbots_tts.tts import speak

from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement


class Speak(AbstractHCMActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.text = parameters.get("text", "").replace("_", " ")
        self.prio = int(parameters.get("prio", 50))

    def perform(self, reevaluate=False):
        speak(self.text, self.blackboard.speak_publisher, self.prio)
        return self.pop()


class Complain(Speak):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.text = random.choice(
            [
                "I am not feeling well.",
                "I am not satisfied.",
                "Referee!!! This was definitely against the rules.",
                "Look at this! This is not how you play soccer.",
                "I am not a toy.",
                "This was definitely a foul.",
                "Do you think this is funny?",
                "Referee!!! Pushing! Pushing! Definitely pushing!",
            ]
        )
