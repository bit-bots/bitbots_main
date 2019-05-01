import rospy
from tf2_geometry_msgs import PoseStamped

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class SendStop(AbstractActionElement):
    """ This stopps the robots walking and directly pops itself  """

    def perform(self, reevaluate=False):
        stand_pose = PoseStamped()
        stand_pose.header.stamp = rospy.Time.now()
        stand_pose.header.frame_id = 'base_footprint'
        stand_pose.pose.orientation.w = 1
        self.blackboard.pathfinding.publish(stand_pose)
        self.pop()


class Stop(AbstractActionElement):
    """ This stopps the robots walking and pops itself after stop has completed """

    def perform(self, reevaluate=False):
        stand_pose = PoseStamped()
        stand_pose.header.stamp = rospy.Time.now()
        stand_pose.header.frame_id = 'base_footprint'
        stand_pose.pose.orientation.w = 1
        self.blackboard.pathfinding.publish(stand_pose)
        #TODO we should wait until the robot really stopped, by looking at the odometry
        self.pop()