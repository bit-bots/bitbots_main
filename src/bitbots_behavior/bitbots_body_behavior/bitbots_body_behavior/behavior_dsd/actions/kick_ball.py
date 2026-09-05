import math

from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_blackboard.capsules.kick_capsule import KickCapsule
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


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

        if not self.blackboard.kick.is_currently_kicking:
            self.pop()


class RLKickTowardsGoal(AbstractKickAction):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self._strength = parameters.get("strength", 2.0)
        self._start_time = None

    def perform(self, reevaluate=False):
        _, _, kick_dir_rad_in_map, _ = self.blackboard.pathfinding.get_map_goal(
            distance=0.5, side_offset=0.0, goal_offset=self.blackboard.config["rl_kick"]["goal_kick_dir_inset"]
        )
        kick_dir_deg_in_map = math.degrees(kick_dir_rad_in_map)
        # transform map to robot relative
        if self._start_time is None:
            self._start_time = self.blackboard.node.get_clock().now()
            self.blackboard.kick.start_rl_kick(kick_dir_deg_in_map, self._strength)

        if not self.blackboard.kick.is_currently_kicking:
            self.pop()

class RLKickForward(AbstractKickAction):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self._strength = parameters.get("strength", 2.0)
        self._start_time = None

    def perform(self, reevaluate=False):
        kick_dir_deg_in_map = 0.0
        # transform map to robot relative
        if self._start_time is None:
            self._start_time = self.blackboard.node.get_clock().now()
            self.blackboard.kick.start_rl_kick(kick_dir_deg_in_map, self._strength)

        if not self.blackboard.kick.is_currently_kicking:
            self.pop()


class RLKickAngleRobot(AbstractKickAction):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self._strength = parameters.get("strength", 2.0)
        self._angle_deg_in_map = parameters.get("angle_deg_in_map", 0.0)
        self._start_time = None

    def perform(self, reevaluate=False):
        # transform map to robot relative
        if self._start_time is None:
            self._start_time = self.blackboard.node.get_clock().now()
            self.blackboard.kick.start_rl_kick(self._angle_deg_in_map, self._strength)

        if not self.blackboard.kick.is_currently_kicking:
            self.pop()
