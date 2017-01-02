# -*- coding:utf-8 -*-
from bitbots.modules.abstract.abstract_stack_element import AbstractStackElement
from bitbots.modules.abstract.abstract_module import debug_m


class AbstractActionModule(AbstractStackElement):
    def __repr__(self):
        """
        Wir kürzen die Repräsentation ab, ist so kürzer, und sagt
        trotzdem noch genug
        """
        return "<Action: " + \
               self.__class__.__module__.split('.')[-2] \
               + "." + self.__class__.__name__ + ">"
