from abc import ABCMeta
from typing import TYPE_CHECKING

from dynamic_stack_decider.abstract_stack_element import AbstractStackElement

if TYPE_CHECKING:
    from dynamic_stack_decider.dsd import DSD


class AbstractActionElement(AbstractStackElement, metaclass=ABCMeta):
    """
    One action is similar to a state of an FSM.
    As in this case, the system stays in this state in contrast to the decision elements which are called only for determining the active action.
    It defines the actions which the robot does, for example performing a kick.
    Another example is an action which takes care of going to the ball, the action remains on top of the stack until the ball is reached.
    The action only makes decisions which are necessary for its purpose, like some adjustments to the kicking movement.
    Actions do not push further elements on the stack but command actions on lower-level modules like new movement goals.
    If the action is complete, it can remove itself from the stack by performing a pop command.
    """

    def __init__(self, blackboard, dsd: "DSD", parameters: dict[str, bool | int | float | str]):
        """
        Constructor of the action element
        :param blackboard: Shared blackboard for data exchange between elements
        :param dsd: The stack decider which has this element on its stack.
        :param parameters: Parameters which serve as arguments to this element
        """
        super().__init__(blackboard, dsd, parameters)
        # Reevaluation can be disabled by setting 'r' or 'reevaluate' to False
        self.never_reevaluate = not parameters.get("r", True) or not parameters.get("reevaluate", True)

    def do_not_reevaluate(self):
        """
        Prohibits the next reevaluate.
        This is useful if you have an action which has to be completed without interruption, e.g. a kick animation.
        """
        self._dsd.set_do_not_reevaluate()

    def repr_dict(self) -> dict:
        """
        Represent this stack element as dictionary which is JSON encodable
        """
        return {
            "type": "action",
            "name": self.name,
            "debug_data": self._debug_data,
        }
