import importlib
import inspect
import json
import pkgutil
import sys
import traceback
from pathlib import Path
from typing import Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from dynamic_stack_decider.abstract_stack_element import AbstractStackElement
from dynamic_stack_decider.logger import get_logger
from dynamic_stack_decider.parser import DsdParser
from dynamic_stack_decider.sequence_element import SequenceElement
from dynamic_stack_decider.tree import (
    AbstractTreeElement,
    ActionTreeElement,
    DecisionTreeElement,
    SequenceTreeElement,
    Tree,
)


def discover_elements(path: str) -> dict[str, AbstractStackElement]:
    """
    Extract all the classes from the files in the given path and return a dictionary containing them

    :param path: The absolute path containing the files that should be registered
    :return: A dictionary with class names as keys and classes as values
    :raises ValueError: if path is not an existing directory or file
    """
    elements = {}
    path = Path(path)
    original_pythonpath = sys.path.copy()

    def discover_module_elements(module_name):
        try:
            if sys.modules.get(module_name, False):
                # reload the module if it is already imported,
                # because we might load a different dsd and the module is
                # always e.g. names 'actions' and therefore not reloaded with the new dsd
                importlib.reload(sys.modules[module_name])
            module = importlib.import_module(module_name)
            # add all classes which are defined directly in the target module (not imported)
            elements.update(
                inspect.getmembers(
                    module,
                    lambda m: inspect.isclass(m)
                    and inspect.getmodule(m) == module
                    and issubclass(m, AbstractStackElement),
                )
            )
        except Exception as e:
            get_logger().error(f"Error while loading class {module_name}: {e}")

    if path.is_file():
        # update PYTHONPATH so that path is importable as a module
        sys.path.append(str(path.parent))
        if path.name == "__init__.py":
            module_name = path.parent.name
        else:
            module_name = path.name.rsplit(".", 1)[0]
        discover_module_elements(module_name)

    elif path.is_dir():
        sys.path.append(str(path.parent))
        # discover elements defined in the modules __init__.py file
        discover_module_elements(path.name)
        # discover elements of all submodules
        for _, module_name, _ in pkgutil.walk_packages([str(path)], prefix=path.name + "."):
            discover_module_elements(module_name)

    else:
        raise ValueError(f"Path {path} is not a directory or file")

    # restore original PYTHONPATH so that everything stays consistent
    sys.path = original_pythonpath

    return elements


class DSD:
    """
    One decision is defined as the root decision, the starting point.
    Each decision element, which is pushed on the stack, is immediately executed until no further element is pushed.
    Following, each iteration, for each element is checked if it requires to be reevaluated and finally the
     top element of the stack will be executed, usually an action.
    If the outcome of a reevaluated element changes, the entire stack on top of this element will be dropped and the
     stack newly constructed.
    As soon as the action is complete, the element will be popped off the stack and the module underneath will be
     executed in the next iteration.
    If this is a decision, it again pushes a further decision or an action and the new top element will be executed.

    By this structure, it is always visible which action the robot tries to perform and which decisions were made.

    If a new element is pushed on top of the stack, it is directly executed.
    In most cases, the pushing element is completing its execution with the push of another element.
    Any following code will be executed as soon as the stack is not further expanded.
    """

    start_element = None
    stack_exec_index = -1
    stack_reevaluate = False
    do_not_reevaluate = False
    old_representation = ""
    debug_active_action_cache: Optional[str] = None

    def __init__(self, blackboard, debug_topic: str = None, node: Optional[Node] = None):
        """
        :param blackboard: Blackboard instance which will be available to all modules
        :param debug_topic:  Topic on which debug data should be published
        """

        self.blackboard = blackboard
        self.node = node

        self.tree: Optional[Tree] = None
        # The stack is implemented as a list of tuples consisting of the tree element
        # and the actual module instance
        self.stack: list[tuple[AbstractTreeElement, AbstractStackElement]] = []

        self.actions: dict[str, AbstractActionElement] = {}
        self.decisions: dict[str, AbstractDecisionElement] = {}

        # Check if debugging is active
        self.debug_active = debug_topic is not None

        # Create debug publisher if necessary and possible
        if self.debug_active and node is not None:
            get_logger().debug("Debugging is active")
            # Create tree publisher
            debug_tree_topic = f"{debug_topic}/dsd_tree"
            # We use a latched publisher because the tree is only published once most of the time
            latched_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.debug_tree_publisher = node.create_publisher(String, debug_tree_topic, latched_qos)
            get_logger().debug(f"Debugging tree on '{debug_tree_topic}' (latched)")
            # Create stack publisher
            debug_stack_topic = f"{debug_topic}/dsd_stack"
            self.debug_stack_publisher = node.create_publisher(String, debug_stack_topic, 10)
            get_logger().debug(f"Debugging stack on '{debug_stack_topic}'")
            # Publish the currently active action
            debug_current_action_topic = f"{debug_topic}/dsd_current_action"
            self.debug_current_action_publisher = node.create_publisher(String, debug_current_action_topic, 10)
            get_logger().debug(f"Debugging current action on '{debug_current_action_topic}'")

    def register_actions(self, module_path):
        """
        Register every class in a given path as an action
        :param module_path: A path containing files with classes extending AbstractActionElement
        """
        self.actions = {k: v for k, v in discover_elements(module_path).items() if issubclass(v, AbstractActionElement)}

    def register_decisions(self, module_path):
        """
        Register every class in a given path as a decision
        :param module_path: A path containing files with classes extending AbstractDecisionElement
        """
        self.decisions = {
            k: v for k, v in discover_elements(module_path).items() if issubclass(v, AbstractDecisionElement)
        }

    def load_behavior(self, path):
        """
        Load a .dsd file into the behavior to execute it. This should be called after the actions
        and decisions have been loaded.
        :param path: The path to the .dsd file describing the behavior
        :return:
        """
        parser = DsdParser(self.node)
        self.tree = parser.parse(path)
        self._bind_modules(self.tree.root_element)
        self.set_start_element(self.tree.root_element)
        self.debug_publish_tree()

    def _bind_modules(self, element):
        """
        Recursively traverse the tree and bind the registered action and decision classes to
        the corresponding tree elements
        :param element: The starting element
        """
        if isinstance(element, ActionTreeElement):
            assert (
                element.name in self.actions
            ), f'Provided element "{element.name}" was not found in registered actions!'
            element.module = self.actions[element.name]
        elif isinstance(element, DecisionTreeElement):
            assert (
                element.name in self.decisions
            ), f'Provided element "{element.name}" was not found in registered decisions!'
            element.module = self.decisions[element.name]
            for child in element.children.values():
                self._bind_modules(child)
        elif isinstance(element, SequenceTreeElement):
            for action in element.action_elements:
                self._bind_modules(action)
        else:
            raise ValueError(f'Unknown parser tree element type "{type(element)}" for element "{element}"!')

    def _init_element(self, element: AbstractTreeElement) -> AbstractStackElement:
        """Initializes the module belonging to the given element."""
        if isinstance(element, SequenceTreeElement):
            return SequenceElement(self.blackboard, self, element.action_elements, self._init_element)
        else:
            return element.module(self.blackboard, self, element.parameters)

    def set_start_element(self, start_element: AbstractTreeElement):
        """
        This method defines the start element on the stack, which stays always on the bottom of the stack.
        It should be called in __init__.
        """
        self.start_element = start_element
        self.stack = [(self.start_element, self._init_element(self.start_element))]

    def interrupt(self):
        """
        An interrupt is an event which clears the complete stack to reset the behavior.
        In the special case of RoboCup, we use it when the game-state changes, but it can also be used for
        example if the robot is kidnapped or paused.
        In the following iteration, the stack will be newly created starting at the root element.
        """
        if self.stack_reevaluate:
            # we were currently checking preconditions
            # we stop this, so that update() knows that it has to stop
            self.stack_reevaluate = False
        self.stack = [(self.start_element, self._init_element(self.start_element))]

    def update(self, reevaluate: bool = True):
        """
        Calls the element which is currently on top of the stack.
        Before doing this, all preconditions are checked (all decision elements where reevaluate is true).

        :param: reevaluate: Can be set to False to avoid the reevaluation
        """
        try:
            self.debug_publish_stack()
            self.debug_publish_current_action()

            if reevaluate and not self.do_not_reevaluate:
                self.stack_exec_index = 0
                self.stack_reevaluate = True
                for tree_element, instance in self.stack[:-1]:
                    # check all elements except the top one, but not the actions
                    if isinstance(instance, AbstractDecisionElement) and instance.get_reevaluate():
                        result = instance.perform(True)
                        # Check whether this result would be an ELSE case
                        result_is_else = result not in tree_element.children.keys()
                        # Push element if necessary. Necessary means that the result is unequal to the activation reason of
                        # the next element in the stack, i.e. the decision has changed. However we have to account for ELSE!
                        if result_is_else and self.stack[self.stack_exec_index + 1][0].activation_reason == "ELSE":
                            # In this case the result returned by the decision does not match any of its possible results,
                            # therefore it goes in the 'ELSE' category. If the activation reason is 'ELSE', the decision did
                            # not change, that means no change in the stack is necessary.
                            pass
                        elif result != self.stack[self.stack_exec_index + 1][0].activation_reason:
                            # In this case, however, the activation reason actually did change. Therefore, we have to
                            # discard everything in the stack above the current decision and push the new result.
                            for _ in range(self.stack_exec_index + 1, len(self.stack)):
                                self.stack.pop()[1].on_pop()
                            self.stack_reevaluate = False
                            self.push(tree_element.get_child(result))

                        if not self.stack_reevaluate:
                            # We had some external interrupt, we stop here
                            return
                    self.stack_exec_index += 1
                self.stack_reevaluate = False
            # Get the top module
            current_tree_element, current_instance = self.stack[-1]
            if reevaluate:
                # reset flag
                self.do_not_reevaluate = False
            if (
                isinstance(current_instance, AbstractActionElement)
                and current_instance.never_reevaluate
                or isinstance(current_instance, SequenceElement)
                and current_instance.current_action.never_reevaluate
            ):
                # Deactivate reevaluation if action had never_reevaluate flag
                self.set_do_not_reevaluate()
            # Run the top module
            result = current_instance.perform()
            if isinstance(current_instance, AbstractDecisionElement):
                self.push(current_tree_element.get_child(result))
        except Exception:
            get_logger().error(str(traceback.format_exc()))

    def push(self, element: AbstractTreeElement):
        """
        Put a new element on the stack and start it directly.

        This should only be called by the DSD, not from any of the modules

        :param element: The tree element that should be put on top of the stack.
        """
        self.stack.append((element, self._init_element(element)))

        # we call the new element without another reevaluate
        self.update(False)

    def pop(self):
        """
        Removes the element from the stack. The previous element will not be called again.
        """
        if len(self.stack) > 1:
            if self.stack_reevaluate:
                # we are currently reevaluating. we shorten the stack here
                if self.stack_exec_index > 0:
                    ## only shorten stack if it still has one element
                    for _ in range(self.stack_exec_index, len(self.stack)):
                        self.stack.pop()[1].on_pop()
                # stop reevaluating
                self.stack_reevaluate = False
            else:
                if isinstance(self.stack[-1][1], SequenceElement):
                    # If we are in a sequence, only one action should be popped
                    if not self.stack[-1][1].in_last_element():
                        # We are still in the sequence, therefore we do not want to pop the SequenceElement,
                        # only a single element of the sequence
                        # We also do not want to reset do_not_reevaluate because an action in the sequence
                        # may control the stack beyond its own lifetime but in the sequence element's lifetime
                        self.stack[-1][1].pop_one().on_pop()
                        return
                # Remove the last element of the stack
                self.stack.pop()[1].on_pop()

            # We will reevaluate even when the popped element set do_not_reevaluate
            # because no module should control the stack beyond its lifetime
            self.do_not_reevaluate = False

    def set_do_not_reevaluate(self):
        """No reevaluation on next iteration"""
        self.do_not_reevaluate = True

    def get_stack(self):
        """
        Returns the current stack
        """
        return self.stack

    def debug_publish_stack(self):
        """
        Publishes a JSON representation of the current stack
        """
        if self.debug_active and self.debug_stack_publisher.get_subscription_count() != 0:
            # Construct JSON encodable object which represents the current stack
            data = None
            for tree_elem, elem_instance in reversed(self.stack):
                elem_data = elem_instance.repr_dict()
                elem_data["activation_reason"] = tree_elem.activation_reason
                elem_data["next"] = data
                data = elem_data

            msg = String(data=json.dumps(data))
            self.debug_stack_publisher.publish(msg)

    def debug_publish_tree(self):
        """
        Publishes a JSON representation of the parsed tree
        """
        if self.debug_active:
            # Construct JSON encodable object which represents the current stack
            data = self.tree.repr_dict()
            msg = String(data=json.dumps(data))
            self.debug_tree_publisher.publish(msg)

    def debug_publish_current_action(self):
        """
        Publishes the name of the currently active action
        """
        # Check if debugging is active and if there is something on the stack
        if not self.debug_active or len(self.stack) == 0:
            return

        # Get the top element
        stack_top = self.stack[-1][1]
        # Check if it is an action or a sequence element and retrieve the current action
        if isinstance(stack_top, AbstractActionElement):
            current_action = stack_top
        elif isinstance(stack_top, SequenceElement):
            current_action = stack_top.current_action
        else:
            return

        # Only publish if the action changed
        if current_action.name != self.debug_active_action_cache:
            # Publish the name of the current action
            self.debug_current_action_publisher.publish(String(data=current_action.name))
            # Cache the current action name
            self.debug_active_action_cache = current_action.name
