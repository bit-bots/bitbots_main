import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from tf.transformations import euler_from_quaternion


class PathfindingCapsule:
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.active = False
        self.arrive_time = 0
        self.counter = 0
        self.timeout = 30
        self.goal = MoveBaseGoal()
        # Thresholds to determine whether the transmitted goal is a new one
        self.position_threshold = 0.5
        self.orientation_threshold = 30

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

    def call_action(self, action_msg):
        if self._is_new_goal_far_from_old_goal(action_msg):
            self.cancel()
        if self.active or self.counter == 0:
            self.counter += 1
            return
        self.active = True
        self.arrive_time = rospy.get_time() + self.timeout
        self.counter = 0
        print("New Goal!")
        self.goal.target_pose = action_msg
        self.action_client.send_goal(self.goal, done_cb=self.done_cb)

    def done_cb(self, id, msg):
        rospy.loginfo("Arrived at goal!")
        self.active = False
        self.arrive_time = rospy.get_time()

    def cancel(self):
        rospy.loginfo("Canceling old goal!")
        self.active = False

    def is_walking_active(self):
        return self.active
