import numpy as np


class PreviousActionObject:
    def __init__(self, node):
        self._node = node

        self._previous_action: np.ndarray = np.zeros(
            len(self._node.get_parameter("joints.ordered_relevant_joint_names").value), dtype=np.float32
        )

    def set_previous_action(self, new_previous_action):
        self._previous_action = new_previous_action

    def get_previous_action(self):
        return self._previous_action
