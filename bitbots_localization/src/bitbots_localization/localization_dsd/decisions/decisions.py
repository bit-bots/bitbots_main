import rospy
from humanoid_league_msgs.msg import GameState, RobotControlState
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class CheckFallen(AbstractDecisionElement):
    """
    Checks if robot is fallen
    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.robot_control_state == RobotControlState.FALLEN:
            return "FALLEN"
        return "NOT_FALLEN"

    def get_reevaluate(self):
        return True


class CheckFalling(AbstractDecisionElement):
    """
    Checks if robot is falling
    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.robot_control_state == RobotControlState.FALLING:
            return "FALLING"
        return "NOT_FALLING"

    def get_reevaluate(self):
        return True


class CheckGettingUp(AbstractDecisionElement):
    """
    Checks if robot is getting up
    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.robot_control_state == RobotControlState.GETTING_UP:
            return "GETTING_UP"
        return "NOT_GETTING_UP"

    def get_reevaluate(self):
        return True


class GotUpJustNow(AbstractDecisionElement):
    """
    Checks if robot state changed from getting up to something different since the last tick
    """
    
    def __init__(self, blackboard, dsd, parameters=None):
        super(GotUpJustNow, self).__init__(blackboard, dsd, parameters)
        self.last_state_stand_up = False

    def perform(self, reevaluate=False):
        if self.last_state_stand_up and self.blackboard.robot_control_state != RobotControlState.GETTING_UP:
            self.last_state_stand_up = False
            return "YES"

        if self.blackboard.robot_control_state == RobotControlState.GETTING_UP:
            self.last_state_stand_up = True
            
        return "NO"

    def get_reevaluate(self):
        return True



class CheckGameStateReceived(AbstractDecisionElement):
    """
    Checks if gamestate from gamecontroller is received.

    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if not self.blackboard.game_state_recived:
            if not self.blackboard.initialized:
                self.blackboard.initialized = True
                return "NO_GAMESTATE_INIT"
            else:
                return "DO_NOTHING"

        return "GAMESTATE_RECEIVED"

    def get_reevaluate(self):
        return True


class CheckGameState(AbstractDecisionElement):
    """
    Checks which game state we are in

    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.penalized:
            return "PENALTY"
        elif self.blackboard.game_state == 0:
            return "INIT"
        elif self.blackboard.game_state == 2:
            return "SET"
        elif self.blackboard.game_state == 3:
            return "PLAYING"
        return "NO_INFORMATION"

    def get_reevaluate(self):
        return True
