import rospy
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from bitbots_localization_handler.localization_dsd.localization_blackboard import STATE_SHUT_DOWN, STATE_STARTUP, STATE_FALLEN, STATE_NOTFALLEN, STATE_NONGAME, STATE_INGAME, STATE_INIT, STATE_SET, STATE_PLAYING, STATE_PENALTY

class StartLocalization(AbstractDecisionElement):
    """
    Initializes Localization
    """

    def perform(self, reevaluate=False):
        if self.blackboard.shut_down_request:
            self.blackboard.current_state = STATE_SHUT_DOWN
            return "SHUTDOWN"
        else:
            if not reevaluate:
                self.blackboard.current_state = STATE_STARTUP
            return "STARTUP"

    def get_reevaluate(self):
        return True

class CheckFallen(AbstractDecisionElement):
    """
    Checks if robot is fallen
    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.robot_control_state == 2: #fallen
            return "FALLEN"
        return "NOT_FALLEN"

    def get_reevaluate(self):
        return True



class CheckGameStateReceived(AbstractDecisionElement):
    """
    Checks if gamestate from gamecontroller is received.

    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if (not self.blackboard.last_pose_update_time) or self.blackboard.current_time.to_sec() - self.blackboard.last_pose_update_time.to_sec() > self.blackboard.pose_timeout_duration:
            if not self.blackboard.initialized:
                self.blackboard.current_state = STATE_INIT
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

        if self.blackboard.game_state == 0:
            return "INIT"
        elif self.blackboard.game_state == 2:
            return "SET"
        elif self.blackboard.game_state == 3:
            return "PLAYING"
        return "NO_INFORMATION"

    def get_reevaluate(self):
        return True

class CheckInit(AbstractDecisionElement):
    def perform(self):
        self.clear_debug_data()
        return NotImplementedError

class CheckSet(AbstractDecisionElement):
    def perform(self):
        self.clear_debug_data()
        return NotImplementedError

class CheckPlaying(AbstractDecisionElement):
    def perform(self):
        self.clear_debug_data()
        return NotImplementedError

class CheckPenalty(AbstractDecisionElement):
    def perform(self):
        self.clear_debug_data()
        return NotImplementedError

