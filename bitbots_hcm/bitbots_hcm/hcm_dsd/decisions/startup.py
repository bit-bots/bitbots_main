import math

from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement

from humanoid_league_msgs.msg import RobotControlState


class StartHCM(AbstractHCMDecisionElement):
    """
    Initializes HCM.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.is_initial = True

    def perform(self, reevaluate=False):
        if self.blackboard.shut_down_request:
            if self.blackboard.current_state == RobotControlState.HARDWARE_PROBLEM:
                self.blackboard.current_state = RobotControlState.SHUTDOWN
                return "SHUTDOWN_WHILE_HARDWARE_PROBLEM"
            else:
                self.blackboard.current_state = RobotControlState.SHUTDOWN
                return "SHUTDOWN_REQUESTED"
        else:
            if self.is_initial:
                if not self.is_walkready():
                    return "NOT_WALKREADY"
                else:
                    self.is_initial = False
                self.blackboard.current_state = RobotControlState.STARTUP
            return "RUNNING"

    def is_walkready(self):
        """
        We check if any joint is has an offset from the walkready pose which is higher than a threshold
        """

        if self.blackboard.current_joint_state is None or self.blackboard.current_joint_state.name == []:
            self.blackboard.node.get_logger().warn("No joint state received yet, can not check if we are walkready at startup")
            return False

        for i, joint_name in enumerate(self.blackboard.current_joint_state.name):
            if joint_name == "HeadPan" or joint_name == "HeadTilt":
                continue # we dont care about the head position
            if abs(math.degrees(self.blackboard.current_joint_state.position[i]) -
                   self.blackboard.walkready_pose_dict[joint_name]) > self.blackboard.walkready_pose_threshold:
                return False
        return True

    def get_reevaluate(self):
        return True
