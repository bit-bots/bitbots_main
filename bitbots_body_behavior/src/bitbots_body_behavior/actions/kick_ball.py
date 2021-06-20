import math
import numpy as np
import rospy
from bitbots_msgs.msg import KickGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class AbstractKickAction(AbstractActionElement):
    def pop(self):
        self.blackboard.world_model.forget_ball()
        super(AbstractKickAction, self).pop()


class KickBallStatic(AbstractKickAction):
    def __init__(self, blackboard, dsd, parameters=None):
        super(KickBallStatic, self).__init__(blackboard, dsd, parameters)
        if 'foot' not in parameters.keys():
            # usually, we kick with the right foot
            self.kick = 'kick_right'  # TODO get actual name of parameter from some config
        elif 'right' == parameters['foot']:
            self.kick = 'kick_right'  # TODO get actual name of parameter from some config
        elif 'left' == parameters['foot']:
            self.kick = 'kick_left'  # TODO get actual name of parameter from some config
        else:
            rospy.logerr(
                'The parameter \'{}\' could not be used to decide which foot should kick'.format(parameters['foot']))

    def perform(self, reevaluate=False):
        if not self.blackboard.animation.is_animation_busy():
            self.blackboard.animation.play_animation(self.kick)


class KickBallDynamic(AbstractKickAction):
    """
    Kick the ball using bitbots_dynamic_kick
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(KickBallDynamic, self).__init__(blackboard, dsd, parameters)
        if parameters.get('type', None) == 'penalty':
            self.penalty_kick = True
        else:
            self.penalty_kick = False

        self._goal_sent = False
        self.kick_length = rospy.get_param('behavior/body/kick_cost_kick_length')
        self.angular_range = rospy.get_param('behavior/body/kick_cost_angular_range')
        self.max_kick_angle = rospy.get_param('behavior/body/max_kick_angle')
        self.num_kick_angles = rospy.get_param('behavior/body/num_kick_angles')
        # By default, don't reevaluate
        self.never_reevaluate = parameters.get('r', True) and parameters.get('reevaluate', True)

    def perform(self, reevaluate=False):

        if not self.blackboard.kick.is_currently_kicking:
            if not self._goal_sent:
                goal = KickGoal()
                goal.header.stamp = rospy.Time.now()

                # currently we use a tested left or right kick
                goal.header.frame_id = self.blackboard.world_model.base_footprint_frame  # the ball position is stated in this frame

                if self.penalty_kick:
                    goal.kick_speed = 6.7
                    goal.ball_position.x = 0.25
                    goal.ball_position.y = 0.0
                    goal.ball_position.z = 0
                    kick_direction = math.radians(25)
                else:
                    ball_u, ball_v = self.blackboard.world_model.get_ball_position_uv()
                    goal.kick_speed = 1
                    goal.ball_position.x = ball_u
                    goal.ball_position.y = ball_v
                    goal.ball_position.z = 0
                    kick_directions = sorted(np.linspace(
                        -self.max_kick_angle,
                        self.max_kick_angle,
                        num=self.num_kick_angles), key=abs)

                    kick_direction = kick_directions[np.argmin([self.blackboard.world_model.get_current_cost_of_kick(
                        direction=direction,
                        kick_length=self.kick_length,
                        angular_range=self.angular_range)
                        for direction in kick_directions])]
                goal.kick_direction = Quaternion(*quaternion_from_euler(0, 0, kick_direction))

                self.blackboard.kick.kick(goal)
                self._goal_sent = True

            else:
                self.pop()


class KickBallVeryHard(AbstractKickAction):
    def __init__(self, blackboard, dsd, parameters=None):
        super(KickBallVeryHard, self).__init__(blackboard, dsd, parameters)
        if 'foot' not in parameters.keys():
            # usually, we kick with the right foot
            self.hard_kick = 'kick_right'  # TODO get actual name of parameter from some config
        elif 'right' == parameters['foot']:
            self.hard_kick = 'kick_right'  # TODO get actual name of parameter from some config
        elif 'left' == parameters['foot']:
            self.hard_kick = 'kick_left'  # TODO get actual name of parameter from some config
        else:
            rospy.logerr(
                'The parameter \'{}\' could not be used to decide which foot should kick'.format(parameters['foot']))

    def perform(self, reevaluate=False):
        if not self.blackboard.animation.is_animation_busy():
            self.blackboard.animation.play_animation(self.hard_kick)
