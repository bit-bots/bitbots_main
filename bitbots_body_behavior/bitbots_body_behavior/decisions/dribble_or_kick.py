from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class DribbleOrKick(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.orient_threshold = self.blackboard.config["dribble_orient_threshold"]
        self.goal_distance_threshold = self.blackboard.config["dribble_goal_distance_threshold"]
        self.ball_distance_threshold = self.blackboard.config["dribble_ball_distance_threshold"]

        self.kick_length = self.blackboard.config["kick_cost_kick_length"]
        self.max_kick_angle = self.blackboard.config["max_kick_angle"]
        self.angular_range = self.blackboard.config["kick_cost_angular_range"]
        self.num_kick_angles = self.blackboard.config["num_kick_angles"]

        self.dribble_kick_angle = self.blackboard.config["dribble_kick_angle"]

    def perform(self, reevaluate=False):
        """
        Determines whether we want to dribble if the area in front of us is clear, or to kick
        :param reevaluate:
        :return:
        """
        # robot needs to be correctly aligned to ball, so that opponent goal is in front
        goal_angle = abs(self.blackboard.world_model.get_map_based_opp_goal_angle())
        oriented_to_goal = goal_angle < self.orient_threshold
        self.publish_debug_data(f"Orientation to goal (needs <{self.orient_threshold})", goal_angle)

        # no other robots should be in front of the ball. this means the kick with angle 0 would be the best
        best_kick_direction = self.blackboard.costmap.get_best_kick_direction(
            -self.max_kick_angle, self.max_kick_angle, self.num_kick_angles, self.kick_length, self.angular_range
        )
        front_free = -self.dribble_kick_angle < best_kick_direction < self.dribble_kick_angle
        self.publish_debug_data("best kick direction", best_kick_direction)
        self.publish_debug_data("Front free", front_free)

        # we should be not to close to the goal, otherwise kicking makes more sense. only take x axis into account
        goal_distance = abs(
            self.blackboard.world_model.get_map_based_opp_goal_center_xy()[0]
            - self.blackboard.world_model.get_current_position()[0]
        )
        goal_far = goal_distance > self.goal_distance_threshold
        self.publish_debug_data(f"Goal distance (needs >{self.goal_distance_threshold})", goal_distance)

        # ball needs to be close enough
        ball_distance = self.blackboard.world_model.get_ball_distance()
        ball_near = ball_distance < self.ball_distance_threshold
        self.publish_debug_data(f"Ball distance (needs <{self.ball_distance_threshold})", ball_distance)

        if oriented_to_goal and front_free and goal_far and ball_near:
            return "DRIBBLE"
        else:
            return "KICK"

    def get_reevaluate(self):
        return True
