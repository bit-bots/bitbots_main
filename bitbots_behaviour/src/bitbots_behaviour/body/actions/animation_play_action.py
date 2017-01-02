# -*- coding:utf-8 -*-
"""
AnimationPlayAction
^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>


This is a capsuled Action to play a defined animation. The action waits until it is possible to play an animation and
schedules the one given by parameter. Then it waits until the animation is finished and pops.

History:
* 8/17/14: Created (sheepy)

"""
from bitbots.modules.abstract.abstract_action_module import AbstractActionModule


class AnimationPlayAction(AbstractActionModule):
    def __init__(self, animation_to_play):
        AbstractActionModule.__init__(self, animation_to_play)
        self.animation_to_play = animation_to_play
        self.animation_begin = False

    def perform(self, connector, reevaluate=None):
        self.do_not_reevaluate()
        # If the animation is not busy anymore and we already started it do a pop
        if not connector.animation_capsule().is_animation_busy() and self.animation_begin:
            return self.pop()

        # If the animation has not started yet try it
        if not self.animation_begin:
            # Try to play it - this returns true if the animation is scheduled
            if connector.animation_capsule().play_animation(self.animation_to_play):
                self.animation_begin = True
