import math

from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_blackboard.capsules.kick_capsule import KickCapsule
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from rclpy.duration import Duration


class AbstractKickAction(AbstractActionElement):
    blackboard: BodyBlackboard

    def on_pop(self):
        self.blackboard.world_model.forget_ball()
        super().on_pop()


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
        self._direction_deg_map = parameters.get("direction_deg_map", 0.0)
        self._strength = parameters.get("strength", 2.0)
        self._start_time = None

    def perform(self, reevaluate=False):
        # transform map to robot relative
        if self._start_time is None:
            self._start_time = self.blackboard.node.get_clock().now()
            self.blackboard.kick.start_rl_kick(self._direction_deg_map, self._strength)

        elapsed = self.blackboard.node.get_clock().now() - self._start_time
        if elapsed >= Duration(seconds=self.blackboard.config["rl_kick"]["timeout"]):
            self.blackboard.kick.stop_rl_kick()
            self.pop()


class RLKickTowardsGoal(AbstractKickAction):

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self._strength = parameters.get("strength", 2.0)
        self._start_time = None

    def perform(self, reevaluate=False):
        x_dir, y_dir = self.blackboard.costmap.get_gradient_at_field_position(self.blackboard.world_model.get_ball_position_xy())
        costmap_direction_deg_in_world = (math.degrees(math.atan2(y_dir, x_dir)))
        # transform map to robot relative
        if self._start_time is None:
            self._start_time = self.blackboard.node.get_clock().now()
            self.blackboard.kick.start_rl_kick(costmap_direction_deg_in_world,
                                               self._strength)

        elapsed = self.blackboard.node.get_clock().now() - self._start_time
        if elapsed >= Duration(
                seconds=self.blackboard.config["rl_kick"]["timeout"]+self.blackboard.config["rl_kick"]["post_kick_timeout"]+self.blackboard.config["rl_kick"]["walk_delay"]):
            self.blackboard.kick.stop_rl_kick()
            self.pop()
