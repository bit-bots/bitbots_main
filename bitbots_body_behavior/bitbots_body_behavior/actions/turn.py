import math
from typing import Optional

from bitbots_blackboard.blackboard import BodyBlackboard
from geometry_msgs.msg import PoseStamped, Twist
from tf_transformations import quaternion_from_euler

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class TurnAround(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: BodyBlackboard

        self.orientation_thresh = parameters.get('thresh', 0.5)
        pose = self.blackboard.world_model.get_current_position()

        if pose is None:
            self.pop()
            return
        x, y, theta = pose

        self.theta = theta + math.pi

        self.pose_msg = self.create_pose_msg(self.blackboard.map_frame, x, y, self.theta)

    def create_pose_msg(self ,frame, x, y, theta):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, theta)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        return pose_msg

    def perform(self, reevaluate=False):
        pose = self.blackboard.world_model.get_current_position()
        if pose is None:
            self.blackboard.node.get_logger().warn("Current position is unknown, trying again next tick")
            return
        theta = pose[2]

        self.blackboard.pathfinding.publish(self.pose_msg)
        if (self.theta - theta + math.tau) % math.tau < self.orientation_thresh:
            self.pop()


class TurnLeft(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: BodyBlackboard
        self.current_rotation_vel = 0.0
        self.max_speed = parameters.get('max_speed', 0.3)

        # Check if the have a duration
        self.duration: Optional[float] = parameters.get('duration', None)
        self.start_time = self.blackboard.node.get_clock().now()

    def perform(self, reevaluate=False):
        # Increase the rotation speed if we are not at max speed
        if self.current_rotation_vel < self.max_speed:
            self.current_rotation_vel += 0.05

        # Create the cmd_vel message
        cmd_vel = Twist()
        cmd_vel.angular.z = self.current_rotation_vel

        # Send the rotation speed
        self.blackboard.pathfinding.direct_cmd_vel_pub.publish(cmd_vel)

        if self.duration is not None:
            # Check if the duration is over
            if (self.blackboard.node.get_clock().now() - self.start_time).nanoseconds / 1e9 > self.duration:
                self.pop()
