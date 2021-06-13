import math
import rospy
from actionlib_msgs.msg import GoalStatus
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class TurnAround(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(TurnAround, self).__init__(blackboard, dsd, parameters)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.blackboard.map_frame

        x, y, theta = self.blackboard.world_model.get_current_position()

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y

        self.theta = theta + math.pi

        quaternion = quaternion_from_euler(0, 0, self.theta)

        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.pose_msg = pose_msg

    def perform(self, reevaluate=False):
        theta = self.blackboard.world_model.get_current_position()[2]

        self.blackboard.pathfinding.publish(self.pose_msg)
        if self.blackboard.pathfinding.status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED] or \
                (self.theta - theta + math.tau) % math.tau < 0.5 :
            print(self.blackboard.pathfinding.status)
            self.pop()
