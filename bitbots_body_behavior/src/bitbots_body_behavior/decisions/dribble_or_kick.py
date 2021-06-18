import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class DribbleOrKick(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.orient_threshold = self.blackboard.config['dribble_orient_threshold']
        self.goal_distance_threshold = self.blackboard.config['dribble_goal_distance_threshold']
        self.ball_distance_threshold = self.blackboard.config['dribble_ball_distance_threshold']

        self.kick_length = rospy.get_param('behavior/body/kick_cost_kick_length')
        self.angular_range = rospy.get_param('behavior/body/kick_cost_angular_range')
        self.max_kick_angle = rospy.get_param('behavior/body/max_kick_angle')
        self.num_kick_angles = rospy.get_param('behavior/body/num_kick_angles')

    def perform(self, reevaluate=False):
        """
        Determines whether we want to dribble if the area in front of us is clear, or to kick
        :param reevaluate:
        :return:
        """
        # robot needs to be correctly aligned to ball, so that opponent goal is in front
        goal_angle = abs(self.blackboard.world_model.get_map_based_opp_goal_angle())
        oriented_to_goal = goal_angle < self.orient_threshold
        self.publish_debug_data(f"Orientation to goal (needs <{self.orient_threshold}", goal_angle)

        # no other robots should be in front of the ball. this means the kick with angle 0 would be the best
        best_kick_direction = self.blackboard.world_model.get_best_kick_direction(-self.max_kick_angle,
                                                                                  self.max_kick_angle,
                                                                                  self.num_kick_angles,
                                                                                  self.kick_length,
                                                                                  self.angular_range)
        front_free = best_kick_direction == 0
        self.publish_debug_data("Front free", front_free)

        # we should be not to close to the goal, otherwise kicking makes more sense
        goal_distance = abs(self.blackboard.world_model.get_map_based_opp_goal_distance())
        goal_far = goal_distance > self.goal_distance_threshold
        self.publish_debug_data(f"Goal distance (needs >{self.goal_distance_threshold}", goal_distance)

        # ball needs to be close enough
        ball_distance = self.blackboard.world_model.get_ball_distance()
        ball_near = ball_distance < self.ball_distance_threshold
        self.publish_debug_data(f"Ball distance (needs <{self.ball_distance_threshold}", ball_distance)

        if oriented_to_goal and front_free and goal_far and ball_near:
            return "DRIBBLE"
        else:
            return "KICK"

    def get_reevaluate(self):
        return True
