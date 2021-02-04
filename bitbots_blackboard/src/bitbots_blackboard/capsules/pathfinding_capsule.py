import rospy
import math

import tf2_ros
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PathfindingCapsule:
    def __init__(self):
        # Thresholds to determine whether the transmitted goal is a new one
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.position_threshold = rospy.get_param('/behavior/body/pathfinding_position_threshold')
        self.orientation_threshold = rospy.get_param('/behavior/body/pathfinding_orientation_threshold')
        self.pathfinding_pub = None  # type: rospy.Publisher
        self.pathfinding_cancel_pub = None  # type: rospy.Publisher
        self.goal = None  # type: PoseStamped
        self.current_pose = None # type: PoseStamped

    def publish(self, msg):
        # type: (PoseStamped) -> None
        map_goal = self.transform_goal_to_map(msg)
        if map_goal:
            self.goal = map_goal
            self.pathfinding_pub.publish(self.fix_rotation(map_goal))

    def transform_goal_to_map(self, msg):
        # type: (PoseStamped) -> PoseStamped
        # transform local goal to goal in map frame
        if msg.header.frame_id == 'map':
            return msg
        else:
            try:
                msg.header.stamp = rospy.Time(0)
                map_goal = self.tf_buffer.transform(msg, rospy.param("map_frame"), timeout=rospy.Duration(0.5))
                e = euler_from_quaternion((map_goal.pose.orientation.x, map_goal.pose.orientation.y,
                                           map_goal.pose.orientation.z, map_goal.pose.orientation.w))
                q = quaternion_from_euler(0, 0, e[2])
                map_goal.pose.orientation.x = q[0]
                map_goal.pose.orientation.y = q[1]
                map_goal.pose.orientation.z = q[2]
                map_goal.pose.orientation.w = q[3]
                map_goal.pose.position.z = 0
                return map_goal
            except Exception as e:
                rospy.logwarn(e)
                return

    def fix_rotation(self, msg):
        # type: (PoseStamped) -> PoseStamped
        # this adds translatory movement to a rotation to fix a pathfinding issue
        if (msg.pose.position.x == 0 and msg.pose.position.y == 0 and
                not (msg.pose.orientation.x == 0 and msg.pose.orientation.y == 0 and msg.pose.orientation.z == 0)):
            msg.pose.position.x = 0.01
            msg.pose.position.y = 0.01
        return msg

    def feedback_callback(self, msg):
        # type: (PoseStamped) -> None
        self.current_pose = msg.feedback.base_position

    def get_goal(self):
        # type: () -> PoseStamped
        return self.goal

    def get_current_pose(self):
        return self.current_pose

    def cancel_goal(self):
        self.pathfinding_cancel_pub.publish(GoalID())
