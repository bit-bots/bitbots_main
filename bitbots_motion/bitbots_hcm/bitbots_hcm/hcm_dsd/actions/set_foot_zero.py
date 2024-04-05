from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement


class SetFootZero(AbstractHCMActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.first_perform = True

    def perform(self, reevaluate=False):
        # Just to be sure, we do not want to reevaluate
        self.do_not_reevaluate()

        # We only want to execute this once
        if self.first_perform:
            # Executing this once is sufficient
            self.first_perform = False

            # Wait for the service to be available and call it
            if self.blackboard.foot_zero_service.wait_for_service(timeout_sec=0.5):
                self.blackboard.foot_zero_service.call_async({})
            else:
                self.blackboard.node.get_logger().warn("No foot zeroing service accessible, will not reset sensors")

            self.pop()
