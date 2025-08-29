from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_blackboard.capsules.kick_capsule import KickCapsule
from bitbots_utils.transforms import quat_from_yaw
from dynamic_stack_decider.abstract_action_element import AbstractActionElement

from bitbots_msgs.action import Kick


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


class KickBallStatic(AbstractKickAction):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        if "foot" not in parameters.keys():
            # usually, we kick with the right foot
            self.kick = "kick_right"  # TODO get actual name of parameter from some config
        elif "right" == parameters["foot"]:
            self.kick = "kick_right"  # TODO get actual name of parameter from some config
        elif "left" == parameters["foot"]:
            self.kick = "kick_left"  # TODO get actual name of parameter from some config
        else:
            self.blackboard.node.get_logger().error(
                "The parameter '{}' could not be used to decide which foot should kick".format(parameters["foot"])
            )
        self.requested = False

    def perform(self, reevaluate=False):
        if not self.requested:
            # Request play animation
            self.blackboard.animation.play_animation(self.kick, False)
            self.requested = True

        if not self.blackboard.animation.is_busy() and self.requested:
            self.pop()


class KickBallDynamic(AbstractKickAction):
    """
    Kick the ball using bitbots_dynamic_kick
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        if parameters.get("type", None) == "penalty":
            self.penalty_kick = True
        else:
            self.penalty_kick = False

        self._goal_sent = False
        self.kick_length = self.blackboard.config["kick_cost_kick_length"]
        self.angular_range = self.blackboard.config["kick_cost_angular_range"]
        self.max_kick_angle = self.blackboard.config["max_kick_angle"]
        self.num_kick_angles = self.blackboard.config["num_kick_angles"]
        self.penalty_kick_angle = self.blackboard.config["penalty_kick_angle"]

    def perform(self, reevaluate=False):
        self.publish_debug_data("Currently Kicking", self.blackboard.kick.is_currently_kicking)
        if not self.blackboard.kick.is_currently_kicking:
            if not self._goal_sent:
                goal = Kick.Goal()
                goal.header.stamp = self.blackboard.node.get_clock().now().to_msg()

                # currently we use a tested left or right kick
                goal.header.frame_id = (
                    self.blackboard.world_model.base_footprint_frame
                )  # the ball position is stated in this frame

                if self.penalty_kick:
                    goal.kick_speed = 6.7
                    goal.ball_position.x = 0.22
                    goal.ball_position.y = 0.0
                    goal.ball_position.z = 0.0
                    goal.unstable = True

                    # only check 2 directions, left and right
                    kick_direction = self.blackboard.costmap.get_best_kick_direction(
                        -self.penalty_kick_angle, self.penalty_kick_angle, 2, self.kick_length, self.angular_range
                    )
                else:
                    ball_u, ball_v = self.blackboard.world_model.get_ball_position_uv()
                    goal.kick_speed = 1.0
                    goal.ball_position.x = ball_u
                    goal.ball_position.y = ball_v
                    goal.ball_position.z = 0.0
                    goal.unstable = False

                    kick_direction = self.blackboard.costmap.get_best_kick_direction(
                        -self.max_kick_angle,
                        self.max_kick_angle,
                        self.num_kick_angles,
                        self.kick_length,
                        self.angular_range,
                    )

                goal.kick_direction = quat_from_yaw(kick_direction)

                self.blackboard.kick.dynamic_kick(goal)
                self._goal_sent = True
            else:
                self.pop()
