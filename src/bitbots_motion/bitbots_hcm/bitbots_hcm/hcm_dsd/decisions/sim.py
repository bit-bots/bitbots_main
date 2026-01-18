from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class Simulation(AbstractHCMDecisionElement):
    """
    Checks if the robot is in simulation mode
    """

    def perform(self, reevaluate=False):
        if self.blackboard.simulation_active:
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return False
