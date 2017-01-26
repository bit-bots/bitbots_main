"""
TestWalkingDynamic
^^^^^^^^^^^^^^^^


Testing the walking using a dynamic pattern
"""
import random

from stackmachine.abstract_action_module import AbstractActionModule


class TestWalkingDynamic(AbstractActionModule):
    def __init__(self, _):
        super(TestWalkingDynamic, self).__init__()
        self.lastmove = [5, 0, 0]

    def perform(self, connector, reevaluate=False):
        self.lastmove[0] = max(-10, min(10, self.lastmove[0] + random.randint(-4, 4)))
        self.lastmove[1] = max(-10, min(10, self.lastmove[1] + random.randint(-4, 4)))
        self.lastmove[2] = max(-10, min(10, self.lastmove[2] + random.randint(-4, 4)))

        connector.walking.start_walking_plain(*self.lastmove)

    def get_reevaluate(self):
        return True
