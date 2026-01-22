from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement


class SetSquat(AbstractHCMActionElement):
    """
    Tells the blackboard that we are squatting or not.
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

        # Check if parameters are valid
        assert "squat" in parameters, "No squat parameter given"
        assert isinstance(parameters["squat"], bool), "Squat parameter is not a boolean"

        # Set state in blackboard
        self.blackboard.in_squat = parameters["squat"]

    def perform(self, reevaluate=False):
        # Our job is done, we can pop
        self.pop()
