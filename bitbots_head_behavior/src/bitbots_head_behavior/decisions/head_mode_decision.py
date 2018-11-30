import rospy
from humanoid_league_msgs.msg import HeadMode

from bitbots_dsd.abstract_decision_element import AbstractDecisionElement


class HeadModeDecision(AbstractDecisionElement):

    @staticmethod
    def _register():
        return ['BALL_MODE', 'POST_MODE', 'BALL_GOAL_TRACKING', 'FIELD_FEATURES',
                'NON_FIELD_FEATURES', 'LOOK_DOWN', 'LOOK_UP', 'LOOK_FORWARD', 'DONT_MOVE']

    def perform(self, reevaluate=False) -> str:
        """
        Map the saved head_mode from blackboard to corresponding decision results
        """
        # Ensure that a head_mode value is set
        if self.blackboard.head_capsule.head_mode is None:
            # configured default value
            self.publish_debug_data('using_default', True)
            head_mode = self.blackboard.config['defaults']['head_mode']
        else:
            head_mode = self.blackboard.head_capsule.head_mode

        # map results to all possible values of head_mode
        if head_mode == HeadMode.BALL_MODE:
            return 'BALL_MODE'
        elif head_mode == HeadMode.POST_MODE:
            return 'POST_MODE'
        elif head_mode == HeadMode.BALL_GOAL_TRACKING:
            return 'BALL_GOAL_TRACKING'
        elif head_mode == HeadMode.FIELD_FEATURES:
            return 'FIELD_FEATURES'
        elif head_mode == HeadMode.NON_FIELD_FEATURES:
            return 'NON_FIELD_FEATURES'
        elif head_mode == HeadMode.LOOK_DOWN:
            return 'LOOK_DOWN'
        elif head_mode == HeadMode.LOOK_UP:
            return 'LOOK_UP'
        elif head_mode == HeadMode.LOOK_FORWARD:
            return 'LOOK_FORWARD'
        elif head_mode == HeadMode.DONT_MOVE:
            return 'DONT_MOVE'

        else:
            raise Exception(f'the set head_mode ({head_mode}) is not known')

    def get_reevaluate(self):
        """
        True

        because we always need to know when the role changes as soon as possible
        """
        return True
