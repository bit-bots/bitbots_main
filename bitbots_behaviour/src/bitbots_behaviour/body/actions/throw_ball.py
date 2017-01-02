# -*- coding:utf-8 -*-
from bitbots.modules.abstract.abstract_action_module import AbstractActionModule
from bitbots.modules.behaviour.body.actions.wait import Wait


class ThrowBall(AbstractActionModule):
    def __init__(self, _):
        super(ThrowBall, self).__init__()
        self.picked = False
        self.throwed = False

    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().set_no_head_movement_at_all()
        self.do_not_reevaluate()
        connector.walking_capsule().stop_walking()
        if not self.picked:
            connector.animation_capsule().play_animation("pickup_new_hands")
            self.picked = True

            return self.push(Wait, 8)

        if not self.throwed and not connector.animation_capsule().is_animation_busy():
            connector.animation_capsule().play_animation("throw_new_hands")
            self.throwed = True
