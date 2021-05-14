import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class AlignedToGoal(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(AlignedToGoal, self).__init__(blackboard, dsd, parameters)
        self.goalpost_safety_distance = self.blackboard.config['goalpost_safety_distance']
        self.field_length = rospy.get_param("field_length")
        self.goal_width = rospy.get_param("goal_width")

    def perform(self, reevaluate=False):
        """
        It is determined if the robot is correctly aligned to kick the ball into the goal.
        """
        current_pose = self.blackboard.world_model.get_current_position()
        current_ball = self.blackboard.world_model.get_ball_position_xy()
        if current_pose is None or current_ball is None:
            return 'NO'

        # calculate the distance of the intersection of a line from the robot through the ball and the goal line.
        dist = abs(
            current_pose[1] +
            (current_pose[1] - current_ball[1]) *
            ((- current_pose[0] + self.field_length / 2) / (current_pose[0] - current_ball[0])))

        self.publish_debug_data("projected_distance_to_goal_center", dist)
        if dist < (self.goal_width / 2) - self.goalpost_safety_distance:
            return 'YES'
        else:
            return 'NO'

    def get_reevaluate(self):
        return True
