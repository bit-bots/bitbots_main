from typing import TYPE_CHECKING, Callable

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_stack_element import AbstractStackElement
from dynamic_stack_decider.tree import AbstractTreeElement, ActionTreeElement

if TYPE_CHECKING:
    from dynamic_stack_decider.dsd import DSD


class SequenceElement(AbstractStackElement):
    """
    A sequence element contains multiple action elements.
    The sequence element executes all the actions consecutively. That means that the first action will be performed
    until it pops itself off the stack. Then, the second action is performed. When the last action is popped, the
    sequence element pops itself too.
    This is not an abstract class to inherit from.
    """

    def __init__(
        self,
        blackboard,
        dsd: "DSD",
        actions: list[ActionTreeElement],
        init_function: Callable[[AbstractTreeElement], AbstractStackElement],
    ):
        """
        :param blackboard: Shared blackboard for data exchange and code reuse between elements
        :param dsd: The stack decider which has this element on its stack.
        :param actions: list of of action tree elements / blueprints for the actions
        :param init_function: A function that initializes an action element creating a stack element from a tree element
        """
        super().__init__(blackboard, dsd, dict())
        # Here we store the 'blueprints' of the actions
        self.actions = actions
        # We store a reference to the function that initializes the action elements based on the tree
        self._init_function = init_function
        # Here we store the actual instances of the active action
        # The action is only initialized when it is the current action
        assert len(actions) > 0, "A sequence element must contain at least one action"
        self.current_action: AbstractActionElement = self._init_function(actions[0])
        # Where we are in the sequence
        self.current_action_index: int = 0

    def perform(self, reevaluate=False):
        """
        Perform the current action of the sequence. See AbstractStackElement.perform() for more information

        :param reevaluate: Ignored for SequenceElements
        """
        # Log the active element
        self.publish_debug_data("Active Element", self.current_action.name)
        # Pass the perform call to the current action
        self.current_action.perform()
        # If the action had debug data, publish it
        if self.current_action._debug_data:
            self.publish_debug_data("Corresponding debug data", self.current_action._debug_data)

    def pop_one(self):
        """
        Pop a single element of the sequence
        """
        assert not self.in_last_element(), (
            "It is not possible to pop a single element when" "the last element of the sequence is active"
        )
        # Save the current action to return it
        popped_action = self.current_action
        # Increment the index to the next action and initialize it
        self.current_action_index += 1
        # We initialize the current action here to avoid the problem described in
        # https://github.com/bit-bots/dynamic_stack_decider/issues/107
        self.current_action = self._init_function(self.actions[self.current_action_index])
        # Return the popped action
        return popped_action

    def on_pop(self):
        """
        This method is called when the sequence is popped from the stack.
        This means that the last element of the sequence was also popped, so
        """
        self.current_action.on_pop()

    def in_last_element(self):
        """Returns if the current element is the last element of the sequence"""
        return self.current_action_index == len(self.actions) - 1

    def repr_dict(self) -> dict:
        """
        Represent this stack element as dictionary which is JSON encodable
        """
        data = {
            "type": "sequence",
            "current_action_index": self.current_action_index,
            "current_action": self.current_action.repr_dict(),
            "debug_data": self._debug_data,
        }
        self.clear_debug_data()
        return data
