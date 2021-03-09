import rospy
from tf2_geometry_msgs import PoseStamped

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class Stop(AbstractActionElement):
    """ This stops the robot's walking and pops itself when the robot stands """

    def perform(self, reevaluate=False):
        self.blackboard.pathfinding.cancel_goal()
        self.pop()


class StandAndWait(AbstractActionElement):
    """ This stops the robots walking and keeps standing """

    def __init__(self, blackboard, dsd, parameters=None):
        super(StandAndWait, self).__init__(blackboard, dsd, parameters)
        self.duration = parameters.get('duration', None)

        self.start_time = rospy.Time.now()

    def perform(self, reevaluate=False):
        if self.duration is not None and \
                (rospy.Time.now() - self.start_time) >= rospy.Duration(self.duration):
            return self.pop()
        stand_pose = PoseStamped()
        stand_pose.header.stamp = rospy.Time.now()
        stand_pose.header.frame_id = 'base_footprint'
        stand_pose.pose.orientation.w = 1
        self.blackboard.pathfinding.publish(stand_pose)
