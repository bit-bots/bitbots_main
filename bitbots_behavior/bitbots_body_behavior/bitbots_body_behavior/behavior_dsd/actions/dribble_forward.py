import numpy as np
from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from geometry_msgs.msg import Twist


class DribbleForward(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.max_speed_x = self.blackboard.config["dribble_max_speed_x"]
        self.max_speed_y = self.blackboard.config["dribble_max_speed_y"]
        self.p = self.blackboard.config["dribble_p"]

    def perform(self, reevaluate=False):
        """
        Dribbles the ball forward. It uses a simple P-controller to publish the corresponding velocities directly.

        :param reevaluate:
        :return:
        """
        # Get the ball relative to the base fottprint
        _, ball_v = self.blackboard.world_model.get_ball_position_uv()

        self.current_speed_y = np.clip(ball_v * self.p, -self.max_speed_y, self.max_speed_y)

        # Stop the pathfinding if it is running for some reason
        self.blackboard.pathfinding.cancel_goal()

        # Publish the velocities
        cmd_vel = Twist()
        cmd_vel.linear.x = self.max_speed_x
        cmd_vel.linear.y = self.current_speed_y
        cmd_vel.angular.z = 0.0
        self.blackboard.pathfinding.direct_cmd_vel_pub.publish(cmd_vel)
