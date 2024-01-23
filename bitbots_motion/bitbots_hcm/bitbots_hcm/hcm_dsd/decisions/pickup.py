from humanoid_league_speaker.speaker import speak

from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class PickedUp(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently picked up
    """

    def perform(self, reevaluate=False):
        if self.blackboard.visualization_active:
            return "ON_GROUND"
        # check if the robot is currently being picked up. foot have no connection to the ground,
        # but robot is more or less upright (to differentiate from falling)
        if (
            self.blackboard.pressure_sensors_installed
            and not self.blackboard.simulation_active
            and sum(self.blackboard.pressures) < 10
            and abs(self.blackboard.smooth_accel[0]) < self.blackboard.pickup_accel_threshold
            and abs(self.blackboard.smooth_accel[1]) < self.blackboard.pickup_accel_threshold
        ):
            if not reevaluate:
                speak("Picked up!", self.blackboard.speak_publisher, priority=50)
            # we do an action sequence to go to walkready and stay in picked up state
            return "PICKED_UP"

        # robot is not picked up
        return "ON_GROUND"

    def get_reevaluate(self):
        return True
