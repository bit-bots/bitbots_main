"""
TestWalkingStaic
^^^^^^^^^^^^^^^^

Testing the walking using a fixed pattern
"""
import time

from bitbots_common.stackmachine.abstract_action_module import AbstractActionModule


class TestWalkingStatic(AbstractActionModule):
    def __init__(self, _):
        super(TestWalkingStatic, self).__init__()
        self.starttime = time.time() - 10

    def perform(self, connector, reevaluate=False):
        td = time.time() - self.starttime
        print(td)

        if td < 5:
            connector.walking_capsule().start_walking_plain(10, 0, 0)
        elif td < 10:
            connector.walking_capsule().start_walking_plain(10, 10, 0)
        elif td < 15:
            connector.walking_capsule().start_walking_plain(10, -10, 0)
        elif td < 20:
            connector.walking_capsule().start_walking_plain(0, 0, 0)
        elif td < 25:
            connector.walking_capsule().start_walking_plain(-10, 0, 0)
        elif td < 30:
            connector.walking_capsule().start_walking_plain(-10, 5, 0)
        elif td < 35:
            connector.walking_capsule().start_walking_plain(-10, 0, -10)
        elif td < 40:
            connector.walking_capsule().start_walking_plain(0, 0, -10)
        elif td < 45:
            connector.walking_capsule().start_walking_plain(0, 0, 10)
        elif td < 50:
            connector.walking_capsule().start_walking_plain(0, -10, 10)
        elif td < 55:
            connector.walking_capsule().start_walking_plain(8, 5, -5)
        elif td < 60:
            connector.walking_capsule().start_walking_plain(-4, -3, 8)
        else:
            connector.walking_capsule().stop_walking()

    def get_reevaluate(self):
        return True
