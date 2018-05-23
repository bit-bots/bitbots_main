import rospy
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


class PathfindingCapsule:
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.active = False
        self.counter = 0

    def call_action(self, action_msg):
        if self.active or self.counter == 0:
            self.counter += 1
            return
        self.active = True
        self.counter = 0
        print("New Goal!")
        goal = MoveBaseGoal()
        goal.target_pose = action_msg
        self.action_client.send_goal(goal, done_cb=self.done_cb)

    def done_cb(self, id, msg):
        rospy.loginfo(msg)
        rospy.loginfo(id)
        self.active = False

    def is_walking_active(self):
        return self.active
