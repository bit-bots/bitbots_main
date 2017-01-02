# -*- coding:utf-8 -*-
"""
FocusGoal
^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:

* 4.3.15 erstellt

Just focusing the goal.

"""

from bitbots.modules.abstract.abstract_action_module import AbstractActionModule


class FocusEnemyGoal(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().schedule_enemy_goal_tracking()
