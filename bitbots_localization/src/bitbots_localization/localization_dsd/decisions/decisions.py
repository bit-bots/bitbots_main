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


class CheckPickup(AbstractDecisionElement):
    """
    Checks if robot is picked up
    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.robot_control_state == RobotControlState.PICKED_UP:
            self.blackboard.last_state_pickup = True
            return "UP"
        else:
            if self.blackboard.last_state_pickup:
                self.blackboard.last_state_pickup = False
                return "JUST_DOWN"

        return "DOWN"

    def get_reevaluate(self):
        return True


class GettingUpState(AbstractDecisionElement):
    """
    Checks if the robot falls, stands up or is freshly standing
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(GettingUpState, self).__init__(blackboard, dsd, parameters)
        self.get_up_states = [
            RobotControlState.FALLING,
            RobotControlState.FALLEN,
            RobotControlState.GETTING_UP]

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.robot_control_state in self.get_up_states:
            self.blackboard.last_state_get_up = True
            return "YES"
        else:
            if self.blackboard.last_state_get_up:
                self.blackboard.last_state_get_up = False
                return "GOTUP"

        return "NO"

    def get_reevaluate(self):
        return True



class CheckGameStateReceived(AbstractDecisionElement):
    """
    Checks if gamestate from gamecontroller is received.

    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if not self.blackboard.gamestate.received_gamestate():
            if not self.blackboard.initialized:
                self.blackboard.initialized = True
                return "NO_GAMESTATE_INIT"
            else:
                return "DO_NOTHING"

        return "GAMESTATE_RECEIVED"

    def get_reevaluate(self):
        return True


class GameStateDecider(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GameStateDecider, self).__init__(blackboard, dsd, parameters)
        self.game_states = {
            0: 'INITIAL',
            1: 'READY',
            2: 'SET',
            3: 'PLAYING',
            4: 'FINISHED',
        }

    def perform(self, reevaluate=False):
        """
        Translates GameState in Blackboard into DSD Answer
        :param reevaluate:
        :return:
        """
        game_state_number = self.blackboard.gamestate.get_gamestate()

        if game_state_number == GameState.GAMESTATE_INITAL:
            return "INITIAL"
        elif game_state_number == GameState.GAMESTATE_READY:
            return "READY"
        elif game_state_number == GameState.GAMESTATE_SET:
            return "SET"
        elif game_state_number == GameState.GAMESTATE_PLAYING:
            return "PLAYING"
        elif game_state_number == GameState.GAMESTATE_FINISHED:
            return "FINISHED"

    def get_reevaluate(self):
        """
        Game state can change during the game
        """
        return True


class SecondaryStateDecider(AbstractDecisionElement):
    """
    Decides in which secondary state the game is currently in. The mode of the secondary state is handled in the
    game controller receiver, so the behavior does ont need to deal with this.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(SecondaryStateDecider, self).__init__(blackboard, dsd, parameters)
        self.secondary_game_states = {
            0: 'NORMAL',
            1: 'PENALTYSHOOT',  # should not happen during halftime or extra time
            2: 'OVERTIME',
            3: 'TIMEOUT',
            4: 'DIRECT_FREEKICK',
            5: 'INDIRECT_FREEKICK',
            6: 'PENALTYKICK',
            7: 'CORNER_KICK',
            8: 'GOAL_KICK',
            9: 'THROW_IN',
        }

    def perform(self, reevaluate=False):
        state_number = self.blackboard.gamestate.get_secondary_state()
        # todo this is a temporary hack to make GUI work
        if state_number == GameState.STATE_NORMAL:
            return "NORMAL"
        elif state_number == GameState.STATE_PENALTYSHOOT:
            return "PENALTYSHOOT"
        elif state_number == GameState.STATE_OVERTIME:
            return "OVERTIME"
        elif state_number == GameState.STATE_TIMEOUT:
            return "TIMEOUT"
        elif state_number == GameState.STATE_DIRECT_FREEKICK:
            return "DIRECT_FREEKICK"
        elif state_number == GameState.STATE_INDIRECT_FREEKICK:
            return "INDIRECT_FREEKICK"
        elif state_number == GameState.STATE_PENALTYKICK:
            return "PENALTYKICK"
        elif state_number == GameState.STATE_CORNER_KICK:
            return "CORNER_KICK"
        elif state_number == GameState.STATE_GOAL_KICK:
            return "GOAL_KICK"
        elif state_number == GameState.STATE_THROW_IN:
            return "THROW_IN"

    def get_reevaluate(self):
        """
        Secondary game state can change during the game
        """
        return True


class SecondaryStateTeamDecider(AbstractDecisionElement):
    """
    Decides if our team or the other team is allowed to execute the secondary state.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(SecondaryStateTeamDecider, self).__init__(blackboard, dsd)
        self.team_id = self.blackboard.gamestate.get_team_id()

    def perform(self, reevaluate=False):
        state_number = self.blackboard.gamestate.get_secondary_state()
        # we have to handle penalty shoot differently because the message is strange
        if state_number == GameState.STATE_PENALTYSHOOT:
            if self.blackboard.gamestate.has_kickoff():
                return 'OUR'
            return 'OTHER'
        else:
            if self.blackboard.gamestate.get_secondary_team() == self.team_id:
                return 'OUR'
            return 'OTHER'

    def get_reevaluate(self):
        """
        Secondary state Team can change during the game
        """
        return True


class CheckPenalized(AbstractDecisionElement):
    """
    Decides if we are penalized.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(CheckPenalized, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if self.blackboard.gamestate.get_is_penalized():
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        """
        The state can change during the game
        """
        return True