# -*- coding:utf-8 -*-
"""
KickBall
^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>
"""
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
from bitbots_body_behaviour.actions.go_to import Stand
import rospy


class KickBall(AbstractActionElement):
    """
    Kickt nach dem Ball, bekommt im init_data die Information ob links oder rechts gekickt werden soll
    Kicks the ball, gets in init_data the information about the side.
    """

    def __init__(self, connector, args):
        super(KickBall, self).__init__(connector)
        self.side = args
        self.begin = rospy.get_time()
        self.right_kick = connector.animation.config["Kicks"]["rightKick"]
        self.left_kick = connector.animation.config["Kicks"]["leftKick"]
        self.right_kick_strong = connector.animation.config["Kicks"]["rightKickStrong"]
        self.left_kick_strong = connector.animation.config["Kicks"]["leftKickStrong"]
        self.right_side_kick = connector.animation.config["Kicks"]["rightKickSide"]
        self.left_side_kick = connector.animation.config["Kicks"]["leftKickSide"]
        self.anim_begin = False

    def perform(self, connector, reevaluate=False):
        self.do_not_reevaluate()

        #if rospy.get_time() - self.begin > 3.5:  # wait one moment
        self.do_not_reevaluate()  # dont interrrupt the kick

        connector.blackboard.set_one_time_kicked(True)
        # todo make a better behaviour that looks if the ball realy moved

        if not connector.animation.is_animation_busy() and self.anim_begin:
            # if the animation was performed, jump one level higher
            return self.interrupt()

        self.anim_begin = True

        self.push(Stand)
        if self.side == "RIGHT_KICK":
            connector.animation.play_animation(self.right_kick)
        elif self.side == "LEFT_KICK":
            connector.animation.play_animation(self.left_kick)
        elif self.side == "RIGHT_KICK_STRONG":
            connector.animation.play_animation(self.right_kick_strong)
        elif self.side == "LEFT_KICK_STRONG":
            connector.animation.play_animation(self.left_kick_strong)
        elif self.side == "LEFT_SIDE_KICK":
            connector.animation.play_animation(self.left_side_kick)
        elif self.side == "RIGHT_SIDE_KICK":
            connector.animation.play_animation(self.right_side_kick)
        else:
            raise NotImplementedError("This kick does not exist")
