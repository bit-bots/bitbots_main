
from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf2_geometry_msgs import PoseStamped


class GoToFormationPosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame

        optimal_positioning = self.blackboard.positioning.get_formation_assignment()
        own_position = optimal_positioning[self.blackboard.gamestate.get_own_id()]
        pose = own_position["goal_pose"]
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.orientation.w = pose[2]

        if self.latched:
            # Cancel the path planning if it is running
            self.blackboard.pathfinding.cancel_goal()
            # need to keep publishing this since path planning publishes a few more messages
            self.blackboard.pathfinding.stop_walk()

        current_pose = self.blackboard.world_model.get_current_position_pose_stamped()

        if current_pose is None or pose is None:
            return self.blackboard.pathfinding.publish(pose_msg)

        current_orientation = euler_from_quaternion(numpify(current_pose.pose.orientation))
        goal_orientation = euler_from_quaternion(numpify(goal_pose.pose.orientation))
        angle_to_goal_orientation = abs(math.remainder(current_orientation[2] - goal_orientation[2], math.tau))
        self.publish_debug_data("current_orientation", current_orientation[2])
        self.publish_debug_data("goal_orientation", goal_orientation[2])
        self.publish_debug_data("angle_to_goal_orientation", angle_to_goal_orientation)

        distance = np.linalg.norm(numpify(goal_pose.pose.position) - numpify(current_pose.pose.position))
        self.publish_debug_data("distance", distance)
        if distance < self.threshold and angle_to_goal_orientation < self.orientation_threshold:
            self.latched = self.latch  # Set it to true if we always want to return YES in the future
            return "YES"
        return "NO"
