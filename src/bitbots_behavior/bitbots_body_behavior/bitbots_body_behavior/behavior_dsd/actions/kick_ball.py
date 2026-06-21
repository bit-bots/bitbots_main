from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_blackboard.capsules.kick_capsule import KickCapsule
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_blackboard.capsules.pathfinding_capsule import BallGoalType
from rclpy.duration import Duration
from tf_transformations import euler_from_quaternion
from ros2_numpy import numpify


from bitbots_msgs.msg import HeadMode


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


class RLKick(AbstractKickAction):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self._duration = float(parameters.get("duration", 1.0))
        ball_goal = self.blackboard.pathfinding.get_ball_goal(
            BallGoalType.MAP, 0.1, 0.0
        )
        self._direction = euler_from_quaternion(numpify(ball_goal.pose.orientation))[2]
        # Time spent looking down at the feet/ball before the kick motion is actually started.
        self._prep_start_time = None
        self._kick_start_time = None

    def perform(self, reevaluate=False):
        # Phase 1: look down at the feet/ball shortly before kicking
        if self._prep_start_time is None:
            self._prep_start_time = self.blackboard.node.get_clock().now()
            self.blackboard.misc.set_head_duty(HeadMode.LOOK_AT_FEET)
            return

        # Phase 2: once the head had time to move down, request the kick
        if self._kick_start_time is None:
            self._kick_start_time = self.blackboard.node.get_clock().now()
            self.blackboard.kick.start_rl_kick(self._direction)
            return

        # Phase 3: stop the kick after the configured duration
        elapsed = self.blackboard.node.get_clock().now() - self._kick_start_time
        if elapsed >= Duration(seconds=self._duration):
            self.blackboard.kick.stop_rl_kick()
            self.pop()
