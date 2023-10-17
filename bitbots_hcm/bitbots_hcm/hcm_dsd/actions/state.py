from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement

from bitbots_msgs.msg import RobotControlState


class AbstractRobotState(AbstractHCMActionElement):
    """
    Abstract class to create actions which just stay on top of the stack
    and set the robot state accordingly.
    """
    def get_state(self) -> RobotControlState:
        """
        Returns the state which should be set. This will be implemented by the subclasses.
        """
        raise NotImplementedError

    def perform(self, reevaluate=False):
        # Just to be sure, we do not reevaluate
        self.do_not_reevaluate()
        # Set the state
        self.blackboard.current_state = self.get_state()
        # Our job is done, we can pop
        self.pop()



class RobotStateStartup(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.STARTUP


class RobotStateControllable(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.CONTROLLABLE


class RobotStateWalking(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.WALKING


class RobotStateAnimationRunning(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.ANIMATION_RUNNING


class RobotStatePickedUp(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.PICKED_UP


class RobotStateFalling(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.FALLING


class RobotStateFallen(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.FALLEN


class RobotStateGettingUp(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.GETTING_UP


class RobotStatePenalty(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.PENALTY


class RobotStateRecord(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.RECORD


class RobotStateKicking(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.KICKING


class RobotStateHardwareProblem(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.HARDWARE_PROBLEM


class RobotStateMotorOff(AbstractRobotState):
    def get_state(self) -> RobotControlState:
        return RobotControlState.MOTOR_OFF
