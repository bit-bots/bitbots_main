from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_utils.transforms import quat_from_yaw
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf2_geometry_msgs import PoseStamped


class AbstractGoToPassPosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, accept, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.max_x = self.blackboard.config["supporter_max_x"]
        self.pass_pos_x = self.blackboard.config["pass_position_x"]
        self.pass_pos_y = self.blackboard.config["pass_position_y"]
        self.accept = accept
        self.blocking = parameters.get("blocking", True)

    def perform(self, reevaluate=False):
        # get ball pos
        ball_pos = self.blackboard.world_model.get_ball_position_xy()
        our_pose = self.blackboard.world_model.get_current_position()

        # compute goal
        goal_x = ball_pos[0]
        if self.accept:
            goal_x += self.pass_pos_x

        # Limit the x position, so that we are not running into the enemy goal
        goal_x = min(self.max_x, goal_x)

        # Calculate the two possible y positions for the pass position
        goal_y_options = [ball_pos[1] + self.pass_pos_y, ball_pos[1] - self.pass_pos_y]

        # Filter out out of field positions
        def is_in_field(y):
            field_width = self.blackboard.world_model.field_width
            return -field_width / 2 <= y <= field_width / 2

        goal_y_options_filtered = list(filter(is_in_field, goal_y_options))

        # Fallback: choose the y position that is closest to us if both positions are out of the field (should not happen really, but just in case)
        if len(goal_y_options_filtered) == 0:
            goal_y_options_filtered = goal_y_options

        def distance_to_our_pose(y):
            return abs(y - our_pose[1])

        goal_y = min(goal_y_options_filtered, key=distance_to_our_pose)
        goal_yaw = 0.0

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame
        pose_msg.pose.position.x = float(goal_x)
        pose_msg.pose.position.y = float(goal_y)
        pose_msg.pose.orientation = quat_from_yaw(goal_yaw)
        self.blackboard.pathfinding.publish(pose_msg)

        if not self.blocking:
            self.pop()


class GoToPassPreparePosition(AbstractGoToPassPosition):
    """
    Go to a position 1m left or right from the ball (whichever is closer) as preparation for a pass
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, False, parameters)


class GoToPassAcceptPosition(AbstractGoToPassPosition):
    """
    Go to a position forward of the ball to accept a pass from another robot.
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, True, parameters)
