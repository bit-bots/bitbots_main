import re
from bitbots_dsd.tree import Tree, AbstractTreeElement, DecisionTreeElement, ActionTreeElement


class DSDParser:
    def parse(self, file):
        """
        Parse a .dsd file to a Tree
        :param file: the path to the .dsd file to be parsed
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
        comment = False
        last_indent = 0
        lnr = 0
        with open(file, 'r') as bfile:
            for line in bfile:
                lnr += 1
                line = line.rstrip()
                if not line:
                    continue
                if '**//' in line:
                    # TODO: the line with the block comment end should not be ignored
                    comment = False
                    continue
                if '//**' in line:
                    comment = True
                    continue
                if line.strip().startswith('//'):
                    continue

                if not comment:
                    indent = len(line) - len(line.lstrip())
                    if indent % 4 != 0:
                        raise ParseError('Error parsing line {}: Indent is not a multiple of 4'.format(lnr))

                    line_content = line.lstrip()

                    if indent == 0 and line_content.startswith('-->'):
                        # This is the declaration of the start. Next line contains root element
                        next_is_start = True
                        current_subtree = tree
                        last_indent = indent
                        continue

                    if next_is_start:
                        # This line contains the root element of the main tree
                        next_is_start = False
                        element = self.create_tree_element(line_content, current_tree_element)
                        tree.set_root_element(element)
                        current_tree_element = element

                    if indent == 0 and line_content.startswith('#'):
                        # This is the declaration of a new subtree
                        current_subtree = Tree()
                        subtrees[line_content[1:]] = current_subtree
                        current_tree_element = None
                        last_indent = indent
                        continue

                    if indent < last_indent:
                        # Go layers up, depending on indent difference
                        for _ in range(indent, last_indent, 4):
                            current_tree_element = current_tree_element.parent

                    if re.search(r'\s*-?->\s*', line_content):
                        # Arrow in line, split in decision result and call
                        result, call = re.split(r'\s*-?->\s*', line_content, 1)

                        if call.startswith('#'):
                            # A subtree is called here.
                            subtree_name = call.strip('#')
                            if subtree_name not in subtrees:
                                raise AssertionError('Error parsing line {}: {} not defined'.format(lnr, call))
                            # The root element of the subtree should be placed in this tree position
                            if current_tree_element is None:
                                # The current subtree is empty, set the subtree as its root element
                                current_subtree.set_root_element(subtrees[subtree_name].root_element)
                            else:
                                # Append this subtree in the current position
                                current_tree_element.add_child_element(subtrees[subtree_name].root_element, result)

                        elif call.startswith('@'):
                            # An action is called
                            element = self.create_tree_element(call, current_tree_element)
                            current_tree_element.add_child_element(element, result)

                        elif call.startswith('$'):
                            # A decision is called
                            element = self.create_tree_element(call, current_tree_element)
                            current_tree_element.add_child_element(element, result)
                            current_tree_element = element

                        else:
                            raise ParseError('Error parsing line {}: Element {} is neither an action nor a decision'.format(lnr, call))

                    else:
                        # No arrow, must be the beginning of a new subtree
                        element = self.create_tree_element(line_content, current_tree_element)
                        current_subtree.set_root_element(element)
                        current_tree_element = element

                    last_indent = indent
        return tree

    def create_tree_element(self, name, parent):
        """
        Create a tree element given a name and a parent.
        The method derives the type (Action/Decision) and optional parameters from the name
        :param name: the string describing the element in the dsd description
        :param parent: the parent element of the new element, None for root
        :type parent: DecisionTreeElement
        :return: a TreeElement containing the information given in name
        """
        if name.startswith('$'):
            name = name[1:]
            element = DecisionTreeElement(name, parent)
        elif name.startswith('@'):
            name = name[1:]
            if re.search(r'\s*\+\s*', name):
                # There is a parameter
                name, parameter = re.split(r'\s*\+\s*', name, 1)
                parameter_key, parameter_value = parameter.split(':')
                parameter_dict = {parameter_key: parameter_value}
                element = ActionTreeElement(name, parent, parameter_dict)
            else:
                # There is no parameter
                element = ActionTreeElement(name, parent)
        else:
            raise ParseError()
        return element


class ParseError(AssertionError):
    pass
