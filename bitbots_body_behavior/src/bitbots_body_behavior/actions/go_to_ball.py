import rospy
import tf2_ros as tf2
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToBall, self).__init__(blackboard, dsd, parameters)

        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5.0))
        tf_listener = tf2.TransformListener(self.tf_buffer)

    def perform(self, reevaluate=False):
        ball_u, ball_v = self.blackboard.world_model.get_ball_position_uv()
        point = (ball_u, ball_v, self.blackboard.world_model.get_opp_goal_angle_from_ball())

        if not self.blackboard.config['use_move_base']:
            self.blackboard.pathfinding.pub_simple_pathfinding(point[0], point[1])

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'base_footprint'

        pose_msg.pose.position = Point(point[0], point[1], 0)

        try:
            absolute_pose = self.tf_buffer.transform(pose_msg, 'map', timeout=rospy.Duration(0.3))
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            # TODO maybe go to relative position if no TF is found instead?
            rospy.logwarn(e)
            return

        # To have the object we are going to in front of us, go to a point behind it
        absolute_pose.pose.position.x -= 0.2

        rotation = quaternion_from_euler(0, 0, point[2])
        absolute_pose.pose.orientation = Quaternion(*rotation)

        self.blackboard.pathfinding.call_action(absolute_pose)
