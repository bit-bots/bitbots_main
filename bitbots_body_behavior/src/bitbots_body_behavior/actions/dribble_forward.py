import numpy as np
import rospy
import tf2_ros as tf2
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class DribbleForward(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.max_speed_x = self.blackboard.config['dribble_max_speed_x']
        self.max_speed_y = self.blackboard.config['dribble_max_speed_y']
        self.p = self.blackboard.config['dribble_p']
        self.accel_x = self.blackboard.config['dribble_accel_x']

        self.current_speed_x = 0
        self.current_speed_y = 0

    def perform(self, reevaluate=False):
        """
        Dribbles the ball forward. It uses a simple P-controller to publish the corresponding velocities directly.

        :param reevaluate:
        :return:
        """
        ball_u, ball_v = self.blackboard.world_model.get_ball_position_uv()

        # todo compute yaw speed based on how we are aligned to the goal

        # increase x if ball is in the center, decrease otherwise
        if abs(ball_v) < 0.1:
            x_speed = self.current_speed_x + self.accel_x
        else:
            x_speed = self.current_speed_x - self.accel_x
        self.current_speed_x = np.clip(x_speed, 0, self.max_speed_x)

        # give more speed in y direction based on ball position
        y_speed = ball_v * self.p
        self.current_speed_y = np.clip(y_speed, -self.max_speed_y, self.max_speed_y)

        cmd_vel = Twist()
        cmd_vel.linear.x = self.current_speed_x
        cmd_vel.linear.y = self.current_speed_y
        cmd_vel.angular.z = 0
        self.blackboard.pathfinding.direct_cmd_vel_pub.publish(cmd_vel)
