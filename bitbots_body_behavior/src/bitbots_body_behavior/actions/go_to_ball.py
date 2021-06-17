import rospy
import tf2_ros as tf2
import math
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from actionlib_msgs.msg import GoalStatus

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToBall, self).__init__(blackboard, dsd, parameters)

        if 'target' not in parameters.keys():
            rospy.logerr('The parameter "target" could not be used to decide whether map information is accesible')
        else:
            self.target = parameters['target']

        self.blocking = parameters.get('blocking', True)
        self.distance = parameters.get('distance', self.blackboard.config['ball_approach_dist'])
        self.goal_width = rospy.get_param("goal_width", 2)

    def perform(self, reevaluate=False):

        pose_msg = self.blackboard.pathfinding.get_ball_goal(self.target, self.distance, self.goal_width)
        self.blackboard.pathfinding.publish(pose_msg)

        approach_marker = Marker()
        approach_marker.pose.position.x = self.distance
        approach_marker.type = Marker.SPHERE
        approach_marker.action = Marker.MODIFY
        approach_marker.id = 1
        color = ColorRGBA()
        color.r = 1.0
        color.g = 1.0
        color.b = 1.0
        color.a = 1.0
        approach_marker.color = color
        approach_marker.lifetime = rospy.Duration(nsecs=0.5)
        scale = Vector3(0.2, 0.2, 0.2)
        approach_marker.scale = scale
        approach_marker.header.stamp = rospy.Time.now()
        approach_marker.header.frame_id = self.blackboard.world_model.base_footprint_frame

        self.blackboard.pathfinding.approach_marker_pub.publish(approach_marker)

        if self.blackboard.pathfinding.status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED] or not self.blocking:
            self.pop()
