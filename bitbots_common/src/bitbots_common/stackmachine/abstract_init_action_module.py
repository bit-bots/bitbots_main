# -*- coding:utf-8 -*-
"""
AbstractInitStepActionModule
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Sheepy <sheepy@informatik.uni-hamburg.de>


"""
from bitbots_common.stackmachine.abstract_action_module import AbstractActionModule


class AbstractInitActionModule(AbstractActionModule):
    """ This is for actions that can have an init method that is called
        once and only once and drains no performance from that time """

    def __init__(self, args=None):
        self.perform_backup = self.perform
        self.perform = self.perform_once

    def perform(self, connector, reevaluate=None):
        raise NotImplementedError("Override this")

    def perform_init(self, connector, reevaluate=None):
        pass

    def perform_once(self, connector, reevaluate=None):
        self.perform_init(connector, reevaluate)
        self.perform_backup(connector, reevaluate)
        self.perform = self.perform_backup
