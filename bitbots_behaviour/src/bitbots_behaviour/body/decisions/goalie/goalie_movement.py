# -*- coding:utf-8 -*-
"""
GoalieMovement
^^^^^^^^^^^^^^

This decides whether the goalie runs to the ball, positions himself in the goal, turns itself to the right
direction or does nothing.

History:
* 20.08.2014: Created(Daniel Speck)
* 05.12.14: Complete Refactor (Marc Bestmann)
"""
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_common.connector.connector import BodyConnector


class GoalieMovement(AbstractDecisionModule):
    def __init__(self,  connector: BodyConnector, _):
        super(GoalieMovement, self).__init__(connector)
        self.toggle_goalie_becomes_fieldie = config["Toggles"]["Goalie"]["goalieGoFieldie"]
        self.toggle_goalie_go_to_ball = config["Toggles"]["Goalie"]["goalieGoToBall"]
        self.toggle_goalie_position_in_goal = config["Toggles"]["Goalie"]["walkInGoal"]
        self.toggle_goalie_check_direction = config["Toggles"]["Goalie"]["checkDirection"]
        self.go_to_ball_velocity = config["Goalie"]["goToBallVelocity"]
        self.go_to_ball_u = config["Goalie"]["goToBallu"]
        self.sideward_move_u_threshold = config["Goalie"]["sidewardMoveUThreshold"]
        self.direction_angle_threshold = config["Goalie"]["directionAngleThreshold"]

    def perform(self, connector, reevaluate=False):

        ball_u = connector.filtered_vision_capsule().get_local_goal_model_ball()[0]
        speed = connector.filtered_vision_capsule().get_ball_speed()
        robot_direction = connector.world_model_capsule().get_current_position()[2]

        # If interrupt from throw comes here we want to wait until its over. Because the vision values will also be bad
        if connector.animation_capsule().is_animation_busy():
            return

        # is it time to be a striker
        if self.toggle_goalie_becomes_fieldie \
                and self.game_situation_needs_fieldie():
            return self.push(BecomeTeamPlayer)

        # Decide if we want to shoot the ball (also known as Manuel-Neuer-Mode)
        if self.toggle_goalie_go_to_ball \
                and speed < self.go_to_ball_velocity \
                and ball_u < self.go_to_ball_u:
            return self.push(BecomeRunningGoalie)

        # check if the goalie was thrown and we have to turn therefore
        if connector.blackboard_capsule().was_thrown():
            return self.push(AfterThrowDecision)

        # check if the goalie is looking in the right direction, because this can change when moving inside the goal
        if self.toggle_goalie_check_direction \
                and abs(robot_direction) < self.direction_angle_threshold:
            return self.push(TurnToAbsoluteDirection, (0, 10))

        # position in goal by sideward movement
        if self.toggle_goalie_position_in_goal \
                and ball_u < self.sideward_move_u_threshold:
            return self.push(PositionInGoal)

            # todo missing else, what to do if not

    def get_reevaluate(self):
        return True

    def game_situation_needs_fieldie(self):
        # todo implement decision based on gamestate to become fieldie
        return False
