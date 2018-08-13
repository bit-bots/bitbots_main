# -*- coding:utf-8 -*-
import json


class AbstractStackElement(object):
    """
    The AbstractStackElement is the basis of all elements on the stack.
    It provides some help functions which should not be overloaded. 
    The work of an element is done in the :func:`perform`.
    Each element which inheritaces the AbstractStackElement can be used as a root element on the stack.
    """
    _behaviour = None
    _init_data = None

    def __init__(self, connector, args=None):
        self.repr_data = {}
        '''This is a dict in which data can be saved that should get represented on a __repr__ call'''

    def setup_internals(self, behaviour, init_data):
        """
        This method initilizes the internal variables and gets called by the stack machine.

        :param behaviour: The stack machine which has this element on its stack.
        :param init_data: The data which was passed on bei __init__.
        """
        self._behaviour = behaviour
        self._init_data = init_data

    def get_init_data(self):
        """
        Polls the init data of the element to check if something changed in between.
        """
        return self._init_data

    def pop(self):
        """
        Help method which pops the element of the stack.
        
        This method should always be called with a return::
            return self.pop()

        If no return is used, further code is executed after the pop, which leads to difficult to debug behavior.

        """
        self._behaviour.pop()    

    def perform(self, connector, reevaluate=False):
        """
        This method is called when the element is on top of the stack. 
        This method has to be overloaded by the implementation!

        :param reevaluate: True if the current method call is a reevaluate of the state. Meaning the modul is not on top of the stack.
        """
        msg = "You schuld overrride perform() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def interrupt(self):
        """
        An interrupt leads to a complete clearing of the stack.
        """
        self._behaviour.interrupt()
    

    def __repr__(self):
        """
        We shorten the representation. It is shorter but has enough information.

        Furthermore, we append the current data of self.rer_data as a JSON.
        """
        shortname = self.__class__.__name__

        data = json.dumps(self.repr_data)

        return "<AbstractStackElement: %s>[%s]" % (shortname, data)

    @staticmethod
    def sign(x):
        return -1 if x < 0 else 1
