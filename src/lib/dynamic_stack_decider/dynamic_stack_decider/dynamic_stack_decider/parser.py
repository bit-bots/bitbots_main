import copy
import re
from typing import Optional

import yaml
from rclpy.node import Node

from dynamic_stack_decider.tree import ActionTreeElement, DecisionTreeElement, Tree


class DsdParser:
    def __init__(self, node: Optional[Node] = None):
        self.node = node

    def parse(self, file_path: str) -> Tree:
        """
        Parse a .dsd file to a Tree

        :param file_path: the path to the .dsd file to be parsed
        :return: a Tree object containing the parsed elements
        """
        # The root tree
        tree = Tree()
        # Dictionary of subtrees that are created
        # The key is the name and the value is the corresponding TreeElement
        subtrees = dict()

        current_subtree = tree
        current_tree_element = None
        next_is_start = False
        next_is_comment = False
        comment = False
        last_indent = 0
        lnr = 0
        with open(file_path) as bfile:
            for line in bfile:
                lnr += 1
                comment = next_is_comment

                line = re.sub(r"//\*\*.*?\*\*//", "", line)  # Block comments starting and ending in the same line

                if "**//" in line:
                    # Block comments ending in this line
                    # This line as well as the following will contain valid code
                    next_is_comment = False
                    comment = False
                    line = re.sub(r".*\*\*//", "", line)
                if "//**" in line:
                    # Block comments starting in this line
                    # This line may contain valid code, the next ones won't
                    next_is_comment = True
                    line = re.sub(r"//\*\*.*", "", line)

                line = re.sub(r"//.*", "", line)  # Line comments

                line = line.rstrip()
                if not line:
                    continue

                if not comment:
                    indent = len(line) - len(line.lstrip())
                    if indent % 4 != 0:
                        raise ParseError(f"Error parsing line {lnr}: Indent is not a multiple of 4")

                    line_content = line.lstrip()

                    if indent == 0 and line_content.startswith("-->"):
                        # This is the declaration of the start. Next line contains root element
                        next_is_start = True
                        current_subtree = tree
                        last_indent = indent
                        continue

                    if next_is_start:
                        # This line contains the root element of the main tree
                        next_is_start = False
                        element = self._create_tree_element(line_content, current_tree_element, lnr)
                        tree.set_root_element(element)
                        current_tree_element = element

                    if indent == 0 and line_content.startswith("#"):
                        # This is the declaration of a new subtree
                        current_subtree = Tree()
                        name = line_content[1:]
                        parameter_list = re.split(r"\s*\+\s*", name)
                        name = parameter_list.pop(0)
                        current_subtree.parameter_list = parameter_list
                        subtrees[name] = current_subtree
                        current_tree_element = None
                        last_indent = indent
                        continue

                    if indent < last_indent:
                        # Go layers up, depending on indent difference
                        for _ in range(indent, last_indent, 4):
                            current_tree_element = current_tree_element.parent

                    if re.search(r"\s*-?->\s*", line_content):
                        # Arrow in line, split in decision result and call
                        result, call = re.split(r"\s*-?->\s*", line_content, maxsplit=1)

                        if call.startswith("#"):
                            # A subtree is called here.
                            if current_tree_element is None:
                                # The current subtree is empty, set the subtree as its root element
                                subtree, _, _ = self._lookup_subtree(call, lnr, subtrees)
                                current_subtree.set_root_element(subtree.root_element)
                            else:
                                # Copy this subtree and place it in the current tree position
                                subtree_copy = self._instantiate_subtree(call, lnr, subtrees)
                                current_tree_element.add_child_element(subtree_copy, result)

                        elif re.search(r"\s*,\s*", call):
                            # A sequence: a chain of actions, optionally ending in a decision or a subtree
                            tokens = re.split(r"\s*,\s*", call)
                            head, tail = self._create_chain(tokens, current_tree_element, lnr, subtrees)
                            current_tree_element.add_child_element(head, result)
                            if tokens[-1].startswith("$"):
                                # A decision at the end of a sequence receives its children from the
                                # following indented lines
                                current_tree_element = tail

                        elif call.startswith("@"):
                            # An action is called
                            element = self._create_tree_element(call, current_tree_element, lnr)
                            current_tree_element.add_child_element(element, result)

                        elif call.startswith("$"):
                            # A decision is called
                            element = self._create_tree_element(call, current_tree_element, lnr)
                            current_tree_element.add_child_element(element, result)
                            current_tree_element = element

                        else:
                            raise ParseError(
                                f"Error parsing line {lnr}: Element {call} is neither an action nor a decision"
                            )

                    else:
                        # No arrow, must be the beginning of a new subtree
                        if re.search(r"\s*,\s*", line_content):
                            tokens = re.split(r"\s*,\s*", line_content)
                            head, tail = self._create_chain(tokens, current_tree_element, lnr, subtrees)
                            current_subtree.set_root_element(head)
                            # A decision at the end of a sequence receives its children from the
                            # following indented lines
                            current_tree_element = tail if tokens[-1].startswith("$") else head
                        else:
                            element = self._create_tree_element(line_content, current_tree_element, lnr)
                            current_subtree.set_root_element(element)
                            current_tree_element = element

                    last_indent = indent
        return tree

    def _extract_parameters(self, token, lnr):
        """
        Extract parameters from a token string in the form of name + key1:value1 + key2:value2
        :param token: the string containing the name and the parameters
        :type token: str
        :param lnr: Line number of the current line (used for error messages)
        :type lnr: int
        :return: the name, a dict of set parameters, a dict of unset parameters
        """
        parameters = re.split(r"\s*\+\s*", token)
        name = parameters.pop(0)
        parameter_dict = dict()
        unset_parameters = dict()
        for parameter in parameters:
            try:
                parameter_key, parameter_value = parameter.split(":")
            except ValueError as e:
                raise ParseError(f"Error parsing line {lnr}: Invalid parameter list") from e

            if " " in parameter_value:
                raise ParseError(
                    f"Error parsing line {lnr}: Parameter values should not contain spaces. Did you forget a comma?"
                )

            if parameter_value.startswith("%"):
                parameter_value = self.node.get_parameter(parameter_value[1:]).value if self.node is not None else None
                parameter_dict[parameter_key] = parameter_value
            elif parameter_value.startswith("*"):
                # This is a reference to the value specified in the subtree
                unset_parameters[parameter_key] = parameter_value[1:]
            else:
                parameter_value = yaml.safe_load(parameter_value)  # universal interpretation of correct datatype
                parameter_dict[parameter_key] = parameter_value
        return name, parameter_dict, unset_parameters

    def _create_tree_element(self, token, parent, lnr):
        """
        Create a tree element given a token and a parent.
        The method derives the type (Action/Decision) and optional parameters from the token
        :param token: the string describing the element in the dsd description
        :param parent: the parent element of the new element, None for root
        :type parent: Union[DecisionTreeElement, ActionTreeElement]
        :param lnr: Line number of the current line (used for error messages)
        :type lnr: int
        :return: a TreeElement containing the information given in token
        """
        name, parameter_dict, unset_parameters = self._extract_parameters(token[1:], lnr)
        if token.startswith("$"):
            element = DecisionTreeElement(name, parent, parameter_dict, unset_parameters)
        elif token.startswith("@"):
            element = ActionTreeElement(name, parent, parameter_dict, unset_parameters)
        else:
            raise ParseError("An element has to start with either $ or @")
        return element

    def _create_chain(self, tokens, parent, lnr, subtrees):
        """
        Create a sequence as a chain of tree elements linked via ``next_element``.

        Every element except the last must be an action, as only actions advance to their successor
        by popping themselves. The last element may be an action, a decision, or a subtree call, which
        allows a decision or a subtree to follow a sequence of actions.

        :param tokens: The call tokens of the sequence (e.g. ``["@A", "@B", "$D"]``)
        :type tokens: list[str]
        :param parent: The element the chain is attached to
        :param lnr: Line number of the current line (used for error messages)
        :type lnr: int
        :param subtrees: The already parsed subtrees, needed to resolve a trailing subtree call
        :type subtrees: dict
        :return: A tuple of the head element (the child of ``parent``) and the tail element of the chain
        """
        head = None
        previous = None
        for i, token in enumerate(tokens):
            is_last = i == len(tokens) - 1
            if is_last and token.startswith("#"):
                # A subtree may terminate a sequence
                element = self._instantiate_subtree(token, lnr, subtrees)
            elif is_last:
                # The last element may be an action or a decision
                element = self._create_tree_element(token, parent, lnr)
            else:
                # Only actions may be followed by another element in a sequence
                if not token.startswith("@"):
                    raise ParseError(
                        f"Error parsing line {lnr}: Only actions may be followed by another element "
                        f"in a sequence, but got '{token}'"
                    )
                element = self._create_tree_element(token, parent, lnr)

            if previous is None:
                head = element
            else:
                previous.next_element = element
            previous = element
        return head, previous

    def _lookup_subtree(self, call, lnr, subtrees):
        """
        Look up the subtree referenced by a ``#``-call and validate that its parameters match.

        :param call: The call token, e.g. ``#SubBehavior + key:value``
        :param lnr: Line number of the current line (used for error messages)
        :param subtrees: The already parsed subtrees
        :return: A tuple of the referenced subtree and the set and unset parameters of the call
        """
        name, parameters, unset_parameters = self._extract_parameters(call.strip("#"), lnr)
        if name not in subtrees:
            raise ParseError(f"Error parsing line {lnr}: {name} not defined")
        subtree = subtrees[name]
        if (set(parameters.keys()) | set(unset_parameters.keys())) != set(subtree.parameter_list):
            raise ParseError(
                "Error parsing line {}: Invalid parameters specified.\n"
                "Available parameters are {}, specified parameters are {}.".format(
                    lnr,
                    set(parameters.keys()) | set(unset_parameters.keys()),
                    set(subtree.parameter_list),
                )
            )
        return subtree, parameters, unset_parameters

    def _instantiate_subtree(self, call, lnr, subtrees):
        """
        Create a deep copy of a referenced subtree with its parameters evaluated.

        :param call: The call token, e.g. ``#SubBehavior + key:value``
        :param lnr: Line number of the current line (used for error messages)
        :param subtrees: The already parsed subtrees
        :return: The root element of the instantiated subtree
        """
        subtree, parameters, unset_parameters = self._lookup_subtree(call, lnr, subtrees)
        subtree_copy = copy.deepcopy(subtree.root_element)
        self._evaluate_subtree_parameters(subtree_copy, parameters, unset_parameters, lnr)
        return subtree_copy

    def _evaluate_subtree_parameters(self, root, parameters, unset_parameters, lnr):
        """
        Resolve the unset parameters of a copied subtree against the parameters given in the call.

        :param root: The root element of the copied subtree
        :param parameters: The set parameters of the subtree call
        :param unset_parameters: The unset (referenced) parameters of the subtree call
        :param lnr: Line number of the current line (used for error messages)
        """
        to_evaluate = [root]
        while len(to_evaluate) > 0:
            current = to_evaluate.pop(0)
            if isinstance(current, DecisionTreeElement):
                to_evaluate.extend(current.children.values())
            elif isinstance(current, ActionTreeElement) and current.next_element is not None:
                # Follow the chain of actions (i.e. a sequence)
                to_evaluate.append(current.next_element)

            for name, reference in current.unset_parameters.items():
                if reference in parameters:
                    current.parameters[name] = parameters[reference]
                elif reference in unset_parameters:
                    current.unset_parameters[name] = unset_parameters[reference]
                else:
                    raise ParseError(f"Error evaluating subtree call in line {lnr}: Unknown reference to {reference}.")


class ParseError(AssertionError):
    """An error during the DSD file parsing occurred"""

    pass
