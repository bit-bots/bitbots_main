import rospy
import tf2_ros as tf2
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler

from dynamic_stack_decider.abstract_action_element import AbstractActionElement

class CircleBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(CircleBall, self).__init__(blackboard, dsd, parameters)

        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5.0))
        tf_listener = tf2.TransformListener(self.tf_buffer)

    def perform(self, reevaluate=False):
        """
        Walks around the ball to get a post into the field of sight to make
        a kick towards the goal possible

        :param reevaluate:
        :return:
        """
        goal_position = self.blackboard.world_model.get_detection_based_goal_position_uv()
        if not goal_position:
            return

        ball_position = self.blackboard.world_model.get_ball_position_uv()
        ball_u, ball_v = ball_position
        point = (ball_u, ball_v, self.blackboard.world_model.get_detection_based_goal_position_uv())

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'base_footprint'

        # ball position
        pose_msg.pose.position = Point(point[0], point[1], 0)

        rotation = quaternion_from_euler(0, 0, 45)  # (hopefully) 45 degrees to the left
        pose_msg.pose.orientation = Quaternion(*rotation)
        pose_msg.pose.position.x -= 0.2  # 20 cm before the ball
        pose_msg.pose.position.y -= 0.2  # 20 cm to the right of the ball

        self.blackboard.pathfinding.publish(pose_msg)

