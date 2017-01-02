# -*- coding:utf-8 -*-
"""
Search
^^^^^^

This Module let the Head do its search, and do nothing.

This is a Symantic Module, it does the same as the :module:`Wait`

History:
''''''''

* ??.??.?? Created (Somebody)

* 06.12.14 Completly changed (Nils Rokita)

"""
from bitbots.modules.abstract.abstract_action_module import AbstractActionModule


class Search(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        # We do nothing here, the head is searching.
        connector.blackboard_capsule().schedule_ball_tracking()
        self.pop()


class StopAndSearch(Search):
    def perform(self, connector, reevaluate=False):
        connector.walking_capsule().stop_walking()
        super(StopAndSearch, self).perform(connector, reevaluate)
