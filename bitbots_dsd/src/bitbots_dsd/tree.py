class Tree:
    """ A tree defining a behaviour, parsed from a .dsd file """
    def __init__(self):
        # The root element of the tree
        self.root_element = None

    def set_root_element(self, element):
        self.root_element = element

    def __repr__(self):
        return repr(self.root_element)


class AbstractTreeElement:
    """
    An element (node) in the tree. Do not use directly,
    use one of DecisionTreeElement and ActionTreeElement instead
    """
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.module = None
        self.parameters = None
        self.activation_reason = None

    def get_child(self, activating_result):
        return None


class DecisionTreeElement(AbstractTreeElement):
    """
    A tree element describing a decision. A decision has children that are executed on a certain result.
    Children can be added with add_child_element
    """
    def __init__(self, name, parent):
        """
        Create a new DecisionTreeElement
        :param name: the class name of the corresponding AbstractDecisionElement
        :param parent: the parent element, None for the root element
        :type parent: DecisionTreeElement
        """
        AbstractTreeElement.__init__(self, name, parent)

        # Dictionary that maps results of the decision to the corresponding child
        self.children = dict()

    def add_child_element(self, element, activating_result):
        """Add a child that will be executed when activating_result is returned"""
        self.children[activating_result] = element
        element.activation_reason = activating_result

    def get_child(self, activating_result):
        """Get the child for a given result"""
        return self.children[activating_result]

    def __repr__(self):
        r = '$' + self.name + ': '
        for result, child in self.children.items():
            r += result + ': {' + repr(child) + '} '
        return r

    def __str__(self):
        return self.name


class ActionTreeElement(AbstractTreeElement):
    """
    A tree element describing an action. An action has optional
    parameters that will be passed to the module on creation
    """
    def __init__(self, name, parent, parameters=None):
        """
        Create a new ActionTreeElement
        :param name: the class name of the corresponding AbstractActionElement
        :param parent: the parent element
        :type parent: DecisionTreeElement
        :param parameters: A dictionary of parameters
        """
        AbstractTreeElement.__init__(self, name, parent)
        self.parameters = parameters

    def __repr__(self):
        return '@{} ({})'.format(self.name, self.parameters)

    def __str__(self):
        return self.name
