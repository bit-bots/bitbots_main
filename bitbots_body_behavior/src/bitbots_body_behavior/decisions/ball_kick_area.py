import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from std_msgs.msg import String


class BallKickArea(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(BallKickArea, self).__init__(blackboard, dsd, parameters)
        self.ball_close_distance = self.blackboard.config['ball_close_distance']
        self.right_kick_min_x = self.blackboard.config['right_kick_min_x']
        self.right_kick_min_y = self.blackboard.config['right_kick_min_y']
        self.right_kick_max_x = self.blackboard.config['right_kick_max_x']
        self.right_kick_max_y = self.blackboard.config['right_kick_max_y']
        self.left_kick_min_x = self.blackboard.config['left_kick_min_x']
        self.left_kick_min_y = self.blackboard.config['left_kick_min_y']
        self.left_kick_max_x = self.blackboard.config['left_kick_max_x']
        self.left_kick_max_y = self.blackboard.config['left_kick_max_y']
        self.viz_publisher = rospy.Publisher('debug/viz_ball_kick_area', String, queue_size=1)

    def perform(self, reevaluate=False):
        """
        Determines with which foot the robot should kick
        :param reevaluate:
        :return:
        """
        ball_data = self.blackboard.world_model.get_ball_position_uv()
        ball_position = ball_data[0], ball_data[1]

        self.publish_debug_data("ball_position", {"u": ball_position[0], "v": ball_position[1]})

        if self.right_kick_min_x <= ball_position[0] <= self.right_kick_max_x and \
           self.right_kick_min_y <= ball_position[1] <= self.right_kick_max_y:
            self.viz_publisher.publish('RIGHT')
            return 'RIGHT'
        if self.left_kick_min_x <= ball_position[0] <= self.left_kick_max_x and \
           self.left_kick_min_y <= ball_position[1] <= self.left_kick_max_y:
            self.viz_publisher.publish('LEFT')
            return 'LEFT'
        # TODO: areas for TURN RIGHT/LEFT/AROUND might be useful.
        self.viz_publisher.publish('FAR')
        return 'FAR'

    def get_reevaluate(self):
        """
        As the position of the ball relative to the robot changes even without actions of the robot,
        this needs to be reevaluated.
        :return: True. Always. Trust me.
        """
        return True
