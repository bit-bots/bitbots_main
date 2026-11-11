from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallInDefensiveArea(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.defensive_area = self.blackboard.config["defensive_area"]

    def perform(self, reevaluate=False):
        """
        Determines whether the ball is in the defensive area of the field as defined in the config.
        :param reevaluate:
        :return:
        """
        ball_position = self.blackboard.world_model.get_ball_position_xy()
        # calculate the x value of the boundary of the defensive area
        defensive_x = (self.defensive_area * self.blackboard.world_model.field_length) - (
            self.blackboard.world_model.field_length / 2
        )
        if ball_position[0] <= defensive_x:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class BallInOwnPercent(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.percent = parameters["p"]

    def perform(self, reevaluate=False):
        """
        Determines whether the ball is in the given percentage of the field towards the own goal.
        :param reevaluate:
        :return:
        """
        ball_position = self.blackboard.world_model.get_ball_position_xy()
        # calculate the x value of the boundary of the defensive area
        defensive_x = ((self.percent / 100.0) * self.blackboard.world_model.field_length) - (
            self.blackboard.world_model.field_length / 2.0
        )
        if ball_position[0] <= defensive_x:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class BallInGoalieZone(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.goalie_zone_x = self.blackboard.config["goalie_zone_x"]
        self.goalie_zone_y = self.blackboard.config["goalie_zone_y"]

    def perform(self, reevaluate=False):
        """
        Determines whether the ball is in the defensive area of the field as defined in the config.
        :param reevaluate:
        :return:
        """
        ball_position = self.blackboard.world_model.get_ball_position_xy()
        # calculate the x value of the boundary of the defensive area
        defensive_x = (self.goalie_zone_x * self.blackboard.world_model.field_length) - (
            self.blackboard.world_model.field_length / 2
        )
        defensive_y_left = (self.goalie_zone_y * self.blackboard.world_model.field_width) - (
            self.blackboard.world_model.field_width / 2
        )
        defensive_y_right = -(
            (self.goalie_zone_y * self.blackboard.world_model.field_width)
            - (self.blackboard.world_model.field_width / 2)
        )
        if (
            ball_position[0] <= defensive_x
            and ball_position[1] <= defensive_y_right
            and ball_position[1] >= defensive_y_left
        ):
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class BallInDemoPenaltyArea(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.demo_penalty_area_x = self.blackboard.config["demo_penalty_area_x"]
        self.demo_penalty_area_x_range = self.blackboard.config["demo_penalty_area_x_range"]
        self.demo_penalty_area_y = self.blackboard.config["demo_penalty_area_y"]
        self.demo_penalty_area_y_range = self.blackboard.config["demo_penalty_area_y_range"]

    def perform(self, reevaluate=False):
        """
        Determines whether the ball is in the defensive area of the field as defined in the config.
        :param reevaluate:
        :return:
        """
        ball_position = self.blackboard.world_model.get_ball_position_xy()
        # calculate the x value of the boundary of the defensive area
        penalty_area_x_upper = (
            self.demo_penalty_area_x * self.blackboard.world_model.field_length
            + self.demo_penalty_area_x_range * self.blackboard.world_model.field_length
        )
        penalty_area_x_lower = (
            self.demo_penalty_area_x * self.blackboard.world_model.field_length
            - self.demo_penalty_area_x_range * self.blackboard.world_model.field_length
        )

        defensive_y_left = (self.demo_penalty_area_y * self.blackboard.world_model.field_width) - (
            self.blackboard.world_model.field_width * self.demo_penalty_area_y_range
        )
        defensive_y_right = -(
            (self.demo_penalty_area_y * self.blackboard.world_model.field_width)
            - (self.blackboard.world_model.field_width * self.demo_penalty_area_y_range)
        )

        if (
            ball_position[0] <= penalty_area_x_upper
            and ball_position[0] >= penalty_area_x_lower
            and ball_position[1] <= defensive_y_right
            and ball_position[1] >= defensive_y_left
        ):
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
