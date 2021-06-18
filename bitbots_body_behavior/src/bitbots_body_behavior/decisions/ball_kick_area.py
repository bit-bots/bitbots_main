import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from std_msgs.msg import String


class BallKickArea(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(BallKickArea, self).__init__(blackboard, dsd, parameters)
        self.kick_x_enter = self.blackboard.config['kick_x_enter']
        self.kick_y_enter = self.blackboard.config['kick_y_enter']
        self.kick_x_leave = self.blackboard.config['kick_x_leave']
        self.kick_y_leave = self.blackboard.config['kick_y_leave']
        self.last_descision = "FAR"

    def perform(self, reevaluate=False):
        """
        Determines with which foot the robot should kick
        :param reevaluate:
        :return:
        """
        # Get the ball position
        ball_position = self.blackboard.world_model.get_ball_position_uv()

        self.publish_debug_data("ball_position", {"u": ball_position[0], "v": ball_position[1]})

        # Check if the ball is in the enter area
        if 0 <= ball_position[0] <= self.kick_x_enter and 0 <= abs(ball_position[1]) <= self.kick_y_enter:
            self.last_descision = 'NEAR'
            return 'NEAR'
        # Check if the ball is in the area between the enter area and the leave area
        elif 0 <= ball_position[0] <= self.kick_x_leave and 0 <= abs(ball_position[1]) <= self.kick_y_leave:
            # Return them explicitly to make the parsing easyer for e.g. the DSD GUI
            if self.last_descision == "FAR":
                return "FAR"
            elif self.last_descision == "NEAR":
                return 'NEAR'
            else:
                rospy.logerr(f"Unknown BallKickArea last return value: {self.last_descision}")
        # We are outside of both areas
        else:
            self.last_descision = 'FAR'
            return 'FAR'

    def get_reevaluate(self):
        """
        As the position of the ball relative to the robot changes even without actions of the robot,
        this needs to be reevaluated.
        :return: True. Always. Trust me.
        """
        return True
