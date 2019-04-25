import rospy
import tf2_ros as tf2
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToBall, self).__init__(blackboard, dsd, parameters)

        if 'target' not in parameters.keys():
            rospy.logerr('The parameter \'{}\' could not be used to decide whether map information is accesible'.format(parameters['target']))
        else:
            self.target = parameters['target']

        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5.0))
        tf_listener = tf2.TransformListener(self.tf_buffer)

    def perform(self, reevaluate=False):
        ball_position = self.blackboard.world_model.get_ball_position_uv()
        if not ball_position:
            return
        ball_u, ball_v = ball_position

        if 'map_goal' == self.target:
            point = (ball_u, ball_v, self.blackboard.world_model.get_map_based_opp_goal_angle_from_ball())
        elif 'detection_goal' == self.target:
            point = (ball_u, ball_v, self.blackboard.world_model.get_detection_based_goal_position_uv())


        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'base_footprint'

        pose_msg.pose.position = Point(point[0], point[1], 0)

        try:
            absolute_pose = self.tf_buffer.transform(pose_msg, 'map', timeout=rospy.Duration(0.3))

            # To have the object we are going to in front of us, go to a point behind it
            absolute_pose.pose.position.x -= 0.2

            rotation = quaternion_from_euler(0, 0, point[2])
            absolute_pose.pose.orientation = Quaternion(*rotation)

            self.blackboard.pathfinding.publish(absolute_pose)

        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            rospy.loginfo('No transform to map frame available, going to relative position instead')
            # TODO add an angle
            pose_msg.pose.orientation.w = 1
            pose_msg.pose.position.x -= 0.2
            self.blackboard.pathfinding.publish(pose_msg)
