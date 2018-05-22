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
import math
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_body_behaviour.body.decisions.goalie.after_throw_decision import AfterThrowDecision
from bitbots_body_behaviour.body.decisions.one_time_kicker.one_time_kicker_decision import OneTimeKickerDecision
from bitbots_body_behaviour.body.decisions.goalie.position_in_goal import PositionInGoal
from bitbots_body_behaviour.body.actions.search import Search
from bitbots_body_behaviour.body.actions.go_to import GoToAbsolutePosition
from bitbots_connector.capsules.blackboard_capsule import DUTY_TEAMPLAYER


class GoalieMovement(AbstractDecisionModule):
    def __init__(self,  connector, _):
        super(GoalieMovement, self).__init__(connector)
        self.toggle_goalie_becomes_fieldie = connector.config["Body"]["Behaviour"]["Toggles"]["Goalie"]["goalieGoFieldie"]
        self.toggle_goalie_go_to_ball = connector.config["Body"]["Behaviour"]["Toggles"]["Goalie"]["goalieGoToBall"]
        self.toggle_goalie_position_in_goal = connector.config["Body"]["Behaviour"]["Toggles"]["Goalie"]["walkInGoal"]
        self.toggle_goalie_check_direction = connector.config["Body"]["Behaviour"]["Toggles"]["Goalie"]["checkDirection"]
        self.go_to_ball_velocity = connector.config["Body"]["Behaviour"]["Goalie"]["goToBallVelocity"]
        self.go_to_ball_u = connector.config["Body"]["Behaviour"]["Goalie"]["goToBallu"]
        self.sideward_move_u_threshold = connector.config["Body"]["Behaviour"]["Goalie"]["sidewardMoveUThreshold"]
        self.direction_angle_threshold = connector.config["Body"]["Behaviour"]["Goalie"]["directionAngleThreshold"]

    def perform(self, connector, reevaluate=False):

        ball_u = connector.world_model.get_ball_position_uv[0]
        speed = connector.world_model.get_ball_speed()
        robot_direction = math.degrees(connector.world_model.get_current_position()[2])

        # If interrupt from throw comes here we want to wait until its over. Because the vision values will also be bad
        if connector.animation.is_animation_busy():
            return

        # is it time to be a striker
        if self.toggle_goalie_becomes_fieldie \
                and self.game_situation_needs_fieldie():
            connector.blackboard.set_goalie_out_of_goal(True)
            connector.blackboard.set_duty(DUTY_TEAMPLAYER)

        # Decide if we want to shoot the ball (also known as Manuel-Neuer-Mode)
        if self.toggle_goalie_go_to_ball \
                and speed < self.go_to_ball_velocity \
                and ball_u < self.go_to_ball_u:
            return self.push(OneTimeKickerDecision)

        # check if the goalie was thrown and we have to turn therefore
        if connector.blackboard.was_thrown():
            return self.push(AfterThrowDecision)

        # check if the goalie is looking in the right direction, because this can change when moving inside the goal
        if self.toggle_goalie_check_direction \
                and abs(robot_direction) < self.direction_angle_threshold:
            current_position = connector.world_model.get_current_position()
            return self.push(GoToAbsolutePosition, (current_position[0], current_position[1], 0))

        # position in goal by sideward movement
        if self.toggle_goalie_position_in_goal \
                and ball_u < self.sideward_move_u_threshold:
            return self.push(PositionInGoal)

        return self.push(Search)

    def get_reevaluate(self):
        return True

    def game_situation_needs_fieldie(self):
        # todo implement decision based on gamestate to become fieldie
        return False
