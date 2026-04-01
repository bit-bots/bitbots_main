import numpy as np


class PreviousActionObject:
    def __init__(self, config):
        self._previous_action: np.ndarray = np.zeros(
            len(config["joints"]["ordered_relevant_joint_names"]), dtype=np.float32
        )

    def set_previous_action(self, new_previous_action):
        self._phase = new_previous_action

    def get_previous_action(self):
        return self._previous_action
