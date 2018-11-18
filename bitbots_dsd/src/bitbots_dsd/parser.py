import re


class DSDParser:
    def parse(self, file):
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
                try:
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
                            raise AssertionError(f'Indent in line {lnr} is not a multiple of 4')

                        line_content = line.lstrip()

                        if indent == 0 and line_content.startswith('-->'):
                            next_is_start = True
                            current_subtree = tree
                            last_indent = indent
                            continue

                        if next_is_start:
                            next_is_start = False
                            element = self.create_tree_element(line_content, current_tree_element)
                            tree.set_root_element(element)
                            current_tree_element = element

                        if indent < last_indent:
                            # Go layers up, depending on indent difference
                            for _ in range(indent, last_indent, 4):
                                current_tree_element = current_tree_element.parent

                        if indent == 0 and line_content.startswith('#'):
                            current_subtree = Tree()
                            subtrees[line_content[1:]] = current_subtree
                            current_tree_element = None

                        if re.search(r'\s*-?->\s*', line_content):
                            # Arrow in line, split in decision result and call
                            result, call = re.split(r'\s*-?->\s*', line_content, 1)

                            if call.startswith('#'):
                                # Handle sub-behaviour
                                subtree_name = call.strip('#')
                                if subtree_name not in subtrees:
                                    raise AssertionError(f'Line {lnr}: {call} not defined')
                                if current_tree_element is None:
                                    current_subtree.set_root_element(subtrees[subtree_name].root_element)
                                else:
                                    current_tree_element.add_child_element(subtrees[subtree_name].root_element, result)

                            if call.startswith('@'):
                                element = self.create_tree_element(call, current_tree_element)
                                current_tree_element.add_child_element(element, result)

                            if call.startswith('$'):
                                element = self.create_tree_element(call, current_tree_element)
                                current_tree_element.add_child_element(element, result)
                                current_tree_element = element

                        else:
                            # No arrow, must be the beginning of a new subtree
                            element = self.create_tree_element(line_content, current_tree_element)
                            current_subtree.set_root_element(element)
                            current_tree_element = element

                        last_indent = indent
                except Exception as e:
                    print('Error parsing line ', lnr, e)
                    # raise e
        return tree

    def create_tree_element(self, name, parent):
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
            element = TreeElement(name, parent)
        return element


class Tree:
    def __init__(self):
        self.root_element = None

    def set_root_element(self, element):
        self.root_element = element

    def __repr__(self):
        return repr(self.root_element)


class TreeElement:
    def __init__(self, name: str, parent):
        self.name = name
        self.parent = parent
        self.module = None

    def get_child(self, activating_result):
        return None

    def __repr__(self):
        return self.name


class DecisionTreeElement(TreeElement):
    def __init__(self, name: str, parent):
        super().__init__(name, parent)

        # Dictionary that maps results of the decision to the corresponding child
        self.children = dict()

    def add_child_element(self, element, activating_result):
        self.children[activating_result] = element

    def get_child(self, activating_result):
        return self.children[activating_result]

    def __str__(self):
        r = '$' + self.name + ': '
        for result, child in self.children.items():
            r += result + ': {' + repr(child) + '} '
        return r

    def __call__(self, *args, **kwargs):
        if self.module is None:
            raise TypeError('Missing module')
        else:
            self.module(*args, **kwargs)


class ActionTreeElement(TreeElement):
    def __init__(self, name: str, parent, parameters=None):
        super().__init__(name, parent)
        self.parameters = parameters

    def __str__(self):
        return f'@{self.name} ({self.parameters})'

    def __call__(self, *args, **kwargs):
        if self.module is None:
            raise TypeError('Missing module')
        else:
            self.module(*args, **kwargs)
