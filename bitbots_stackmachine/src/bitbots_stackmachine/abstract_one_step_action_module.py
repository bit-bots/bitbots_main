# -*- coding:utf-8 -*-
"""
AbstractOneStepActionModule
^^^^^^^^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:

* 13.12.13: Created (Marc)

TODOs:

"""
from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class AbstractOneStepActionModule(AbstractActionModule):
    """ This is for actions that require a direct pop after one act call"""

    def perform(self, connector, reevaluate=None):
        self.perform_one_step(connector)
        return self.pop()

    def perform_one_step(self, connector, reevaluate=None):
        raise NotImplementedError()
