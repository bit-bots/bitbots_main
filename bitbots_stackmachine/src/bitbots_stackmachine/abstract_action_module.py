# -*- coding:utf-8 -*-
import json
from bitbots_stackmachine.abstract_stack_element import AbstractStackElement


class AbstractActionModule(AbstractStackElement):
    def __repr__(self):
        """
        Wir kürzen die Repräsentation ab, ist so kürzer, und sagt
        trotzdem noch genug
        """
        shortname = self.__class__.__module__.split('.')[-2] \
                    + "." + self.__class__.__name__

        data = json.dumps(self.repr_data)

        return "<Action: %s>[%s]" % (shortname, data)
