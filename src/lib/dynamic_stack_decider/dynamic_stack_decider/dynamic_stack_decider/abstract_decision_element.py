from abc import ABCMeta

from dynamic_stack_decider.abstract_stack_element import AbstractStackElement


class AbstractDecisionElement(AbstractStackElement, metaclass=ABCMeta):
    """
    The logic is encapsulated in two types of elements.
    The decision elements define the logical path similar to a behavior tree.
    Corresponding decisions can be as complex as wished, using in the most simple case
    if-else clauses or in a more complicated situation, for example, neural networks.
    Either way, one decision-element should capsule one logical decision and determine
    the following module, without giving any calls to the hardware level.
    One example is the decision if the robot has sufficient knowledge about the
    current ball position, using data from the vision components and the team-communication.
    By using a push method they add further elements to the stack.
    Thereby, each decision element can have other decision or actions as following active elements.
    All decisions can, therefore, be displayed as a decision tree.
    """

    _reevaluate = False

    def repr_dict(self) -> dict:
        """
        Represent this stack element as dictionary which is JSON encodable
        """
        return {
            "type": "decision",
            "name": self.name,
            "debug_data": self._debug_data,
        }

    def get_reevaluate(self):
        """
        Each decision element may define a \textit{reevaluate} criteria.
        If the corresponding method returns true, the element will be executed even if it is in the middle of the stack.
        This way a precondition can be checked, for example, every tenth iteration.
        As an example, every iteration it could be checked if the ball position is known to the robot with a given certainty.
        If the now pushed element is different from the element which is currently above in the stack, the whole
        stack above will be dropped and the newly selected element executed.

        This method returns whether the element should be reevaluated. This means that the element is recomputed
        even if it is not on top of the stack. If the decision pushes a different element than in the original perform,
        the stack above this decision is cleared.
        This can be used to regularly check preconditions, e.g. if the robot knows where the ball is.
        """
        return self._reevaluate
