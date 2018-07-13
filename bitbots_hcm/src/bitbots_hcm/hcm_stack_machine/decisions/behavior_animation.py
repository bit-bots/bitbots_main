from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class BehaviorAnimation(AbstractDecisionElement):
    """
    Decides if the robot is currently wants to play an animation comming from the behavior
    """

    def __init__(self, connector, _):
        super(BehaviorAnimation, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        if connector.hcm.is_external_animation_running():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_ANIMATION_RUNNING)
            return self.push(StayAnimationRunning)
        else:
            return self.push(Walking)

    def get_reevaluate(self):
        return True
