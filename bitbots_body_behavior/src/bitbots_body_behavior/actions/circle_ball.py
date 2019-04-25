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
        # TODO walk around ball but without getting out of good kicking distance
