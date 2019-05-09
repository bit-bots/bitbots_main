import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class PathfindingCapsule:
    def __init__(self):
        # Thresholds to determine whether the transmitted goal is a new one
        self.position_threshold = 0.5
        self.orientation_threshold = 30
        self.pathfinding_pub = None  # type: rospy.Publisher

    def publish(self, msg):
        # type: (PoseStamped) -> None
        # TODO use _is_new_goal_far_from_old_goal
        self.pathfinding_pub.publish(self.fix_rotation(msg))

    def _is_new_goal_far_from_old_goal(self, new_goal_action_msg):
        old_goal = self.goal.target_pose
        old_position = old_goal.pose.position
        old_orientation = old_goal.pose.orientation
        old_orientation = euler_from_quaternion([old_orientation.x, old_orientation.y, old_orientation.z, old_orientation.w])
        new_position = new_goal_action_msg.pose.position
        new_orientation = new_goal_action_msg.pose.orientation
        new_orientation = euler_from_quaternion([new_orientation.x, new_orientation.y, new_orientation.z, new_orientation.w])

        # Calculate distance between the position
        position_distance = math.sqrt((old_position.x - new_position.x) ** 2 + (old_position.y - new_position.y) ** 2)
        orientation_distance = math.degrees(abs(old_orientation[2] - new_orientation[2]))
        return position_distance > self.position_threshold or orientation_distance > self.orientation_threshold

    def fix_rotation(self, msg):
        # type: (PoseStamped) -> PoseStamped
        # this adds translatory movement to a rotation to fix a pathfinding issue
        if (msg.pose.position.x == 0 and msg.pose.position.y == 0 and
                not (msg.pose.orientation.x == 0 and msg.pose.orientation.y == 0 and msg.pose.orientation.z == 0)):
            msg.pose.position.x = 0.01
            msg.pose.position.y = 0.01
        return msg
