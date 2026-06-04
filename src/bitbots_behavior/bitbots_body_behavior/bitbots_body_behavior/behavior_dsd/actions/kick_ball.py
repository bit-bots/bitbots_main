from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_blackboard.capsules.kick_capsule import KickCapsule
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from rclpy.duration import Duration


class AbstractKickAction(AbstractActionElement):
    blackboard: BodyBlackboard

    def pop(self):
        self.blackboard.world_model.forget_ball()
        super().pop()


class WalkKick(AbstractKickAction):
    target: KickCapsule.WalkKickTargets

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        if "foot" not in parameters.keys():
            raise ValueError("No foot specified for walk kick")
        elif "right" == parameters["foot"]:
            self.target = KickCapsule.WalkKickTargets.RIGHT
        elif "left" == parameters["foot"]:
            self.target = KickCapsule.WalkKickTargets.LEFT
        else:
            raise ValueError(f"Invalid foot specified for walk kick: {parameters['foot']}")

    def perform(self, reevaluate=False):
        self.blackboard.kick.walk_kick(self.target)
        self.pop()


# Currently kicking in no specific direction
class RLKick(AbstractKickAction):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self._duration = parameters.get("duration", 3.0)
        self._start_time = None

    def perform(self, reevaluate=False):
        if self._start_time is None:
            self._start_time = self.blackboard.node.get_clock().now()
            self.blackboard.kick.start_rl_kick()

        elapsed = self.blackboard.node.get_clock().now() - self._start_time
        if elapsed >= Duration(seconds=self._duration):
            self.blackboard.kick.stop_rl_kick()
            self.pop()
