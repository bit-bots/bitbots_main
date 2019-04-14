import rospy
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class KickBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(KickBall, self).__init__(blackboard, dsd, parameters)
        if 'foot' not in parameters.keys():
            # usually, we kick with thr right foot
            self.kick = 'kick_right'  # TODO get actual name of parameter from some config
        elif 'right' == parameters['foot']:
            self.kick = 'kick_right'  # TODO get actual name of parameter from some config
        elif 'left' == parameters['foot']:
            self.kick = 'kick_left'  # TODO get actual name of parameter from some config
        else:
            rospy.logerr('The parameter \'{}\' could not be used to decide which foot should kick'.format(parameters['foot']))

    def perform(self, reevaluate=False):
        if not self.blackboard.animation.is_animation_busy():
            self.blackboard.animation.play_animation(self.kick)


class KickBallVeryHard(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.right_hard_kick = 'kick_right_string'  # TODO get actual name of parameter from some config

    def perform(self, reevaluate=False):
        if not self.blackboard.animation.is_animation_busy():
            self.blackboard.animation.play_animation(self.right_hard_kick)
