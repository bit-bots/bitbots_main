import json

import rospy


class AbstractStackElement(object):
    """
    The AbstractStackElement is the basis of all elements on the stack.
    It provides some help functions which should not be overloaded. 
    The work of an element is done in the :func:`perform`.
    Each element which inheritaces the AbstractStackElement can be used as a root element on the stack.
    """
    dsd = None
    _init_data = None

    def __init__(self, blackboard, dsd, parameters=None):
        """
        :param blackboard: Shared blackboard for data exchange between elements
        :param dsd: The stack decider which has this element on its stack.
        :param parameters: Optional parameters which serve as arguments to this element
        """
        self._debug_data = {}
        '''This is a dict in which data can be saved that should get represented on a __repr__ call'''

        self.dsd = dsd
        self.blackboard = blackboard

    def setup_internals(self, behaviour):
        """
        This method initilizes the internal variables and gets called by the stack machine.


        """

    def pop(self):
        """
        Help method which pops the element of the stack.
        
        This method should always be called with a return::
            return self.pop()

        If no return is used, further code is executed after the pop, which leads to difficult to debug behavior.

        """
        self.dsd.pop()

    def perform(self, reevaluate=False):
        """
        This method is called when the element is on top of the stack. 
        This method has to be overloaded by the implementation!

        :param reevaluate: True if the current method call is a reevaluate of the state. Meaning the modul is not on top of the stack.
        """
        msg = "You should override perform() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def interrupt(self):
        """
        An interrupt leads to a complete clearing of the stack.
        """
        self.dsd.interrupt()

    def publish_debug_data(self, label, data):
        """
        Publish debug data. Can be viewed using the stackmachine-visualization

        :type label: str or None
        :type data: dict or list or int or float or str or bool
        """

        if type(data) not in (dict, list, int, float, str, bool):
            rospy.logdebug_throttle(1, "The supplied debug data of type {} is not JSON serializable and will not be published".format(type(data)))
            return

        if label is None:
            label = len(self._debug_data)

        rospy.logdebug('{}    :   {}'.format(label, data))
        self._debug_data[label] = data

    def __repr__(self):
        """
        We shorten the representation. It is shorter but has enough information.

        Furthermore, we append the current data of self.repr_data as a JSON.
        """
        shortname = self.__class__.__name__

        data = json.dumps(self._debug_data)
        self._debug_data = {}

        return ":abstract:{}[{}]".format(shortname, data)

    @staticmethod
    def sign(x):
        return -1 if x < 0 else 1
