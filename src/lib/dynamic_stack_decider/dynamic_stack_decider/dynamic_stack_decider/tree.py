import sys
from typing import Optional


class Tree:
    """A tree defining a behaviour, parsed from a .dsd file"""

    def __init__(self):
        # The root element of the tree
        self.root_element: Optional[AbstractTreeElement] = None
        self.parameter_list = []

    def set_root_element(self, element: "AbstractTreeElement"):
        """
        Set the root element of the tree

        :param element: The root element
        """
        self.root_element = element

    def __repr__(self):
        return repr(self.root_element)

    def repr_dict(self) -> dict:
        """
        Represent this tree as dictionary which is JSON encodable
        """
        return self.root_element.repr_dict() or {}


class AbstractTreeElement:
    """
    An element (node) in the tree. Do not use directly,
    use one of DecisionTreeElement and ActionTreeElement instead
    """

    def __init__(
        self,
        name: str,
        parent: Optional["AbstractTreeElement"],
        parameters: Optional[dict] = None,
        unset_parameters: Optional[dict] = None,
    ):
        self.name = name
        self.parent = parent
        self.parameters = parameters or dict()
        self.unset_parameters = unset_parameters or dict()
        self.module = None
        self.activation_reason = None

    def get_child(self, activating_result: str) -> Optional["AbstractTreeElement"]:
        """
        Get the child that should be activated for the given result.
        This makes only sense for DecisionTreeElements; others return None

        :param activating_result: The result that activates the returned child
        :return: The corresponding child element or None
        """
        return None

    def set_activation_reason(self, reason):
        """Set the result that activated this element"""
        self.activation_reason = reason

    def repr_dict(self) -> dict:
        """
        Represent this tree element as dictionary which is JSON encodable
        """
        return {
            # A stable identifier for this tree element. It is used to match a stack element to the
            # exact tree element it belongs to, which is required because a chain (i.e. a sequence)
            # may contain multiple elements with the same name (e.g. @Wait, @SomeAction, @Wait).
            "id": id(self),
            "name": self.name,
            "parameters": self.parameters,
        }


class DecisionTreeElement(AbstractTreeElement):
    """
    A tree element describing a decision. A decision has children that are executed on a certain result.
    Children can be added with add_child_element
    """

    def __init__(self, name, parent, parameters=None, unset_parameters=None):
        """
        Create a new DecisionTreeElement

        :param name: the class name of the corresponding AbstractDecisionElement
        :param parent: the parent element, None for the root element
        :param parameters: A dictionary of parameters
        :param unset_parameters: A dictionary of parameters that must be set later
        """
        # Call the constructor of the superclass
        super().__init__(name, parent, parameters, unset_parameters)

        # Dictionary that maps results of the decision to the corresponding child
        self.children: dict[str, AbstractTreeElement] = dict()

    def add_child_element(self, element: AbstractTreeElement, activating_result: str):
        """Add a child that will be executed when activating_result is returned"""
        self.children[activating_result] = element
        element.set_activation_reason(activating_result)

    def get_child(self, activating_result: str) -> Optional[AbstractTreeElement]:
        """Get the child for a given result"""
        if not activating_result:
            # give advice about error
            sys.exit("Decision return was None. You probably forgot to return a string in your perform method.")

        if activating_result in self.children.keys():
            return self.children[activating_result]
        elif "ELSE" in self.children.keys():
            return self.children["ELSE"]
        else:
            raise KeyError(f"{activating_result} does not lead to a child of {str(self)} and no ELSE was specified")

    def __repr__(self):
        r = "$" + self.name + f" ({self.parameters}): "
        for result, child in self.children.items():
            r += result + ": {" + repr(child) + "} "
        return r

    def __str__(self):
        return self.name

    def repr_dict(self) -> dict:
        """
        Represent this decision as dictionary which is JSON encodable
        """
        # Get the dict from the superclass
        result = super().repr_dict()
        # Add additional information
        result["type"] = "decision"
        result["children"] = {result: child.repr_dict() for result, child in self.children.items()}
        return result


class ActionTreeElement(AbstractTreeElement):
    """
    A tree element describing an action. An action has optional
    parameters that will be passed to the module on creation.

    An action may point to a ``next_element`` which is activated once the action
    pops itself off the stack. This is how sequences (``@A, @B, @C``) are
    represented: as a chain of actions linked via ``next_element``. The next
    element may be any tree element, so a decision is also allowed to follow an
    action.
    """

    def __init__(self, name, parent, parameters=None, unset_parameters=None):
        """
        Create a new ActionTreeElement
        :param name: the class name of the corresponding AbstractActionElement
        :param parent: the parent element
        :param parameters: A dictionary of parameters
        :param unset_parameters: A dictionary of parameters that must be set later
        """
        super().__init__(name, parent, parameters, unset_parameters)
        # The element that is activated when this action pops itself. None if this
        # action is the last (or only) element in its chain.
        self.next_element: Optional[AbstractTreeElement] = None

    def set_activation_reason(self, reason):
        """Set the activation reason for this action and propagate it along the chain"""
        self.activation_reason = reason
        # All elements of a chain share the activation reason of the head so that a
        # reevaluated decision below the chain does not consider its result changed
        # while the chain is still running.
        if self.next_element is not None:
            self.next_element.set_activation_reason(reason)

    def __repr__(self):
        if self.next_element is not None:
            return f"@{self.name} ({self.parameters}) -> {repr(self.next_element)}"
        return f"@{self.name} ({self.parameters})"

    def __str__(self):
        return self.name

    def repr_dict(self) -> dict:
        """
        Represent this action as dictionary which is JSON encodable
        """
        # Get the dict from the superclass
        result = super().repr_dict()
        # Add additional information
        result["type"] = "action"
        result["next"] = self.next_element.repr_dict() if self.next_element is not None else None
        return result
