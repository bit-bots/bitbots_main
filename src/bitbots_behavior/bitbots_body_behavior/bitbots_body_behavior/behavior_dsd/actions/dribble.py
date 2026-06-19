import numpy as np
from bitbots_blackboard.body_blackboard import BodyBlackboard
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

    def on_pop(self):
        self.blackboard.world_model.forget_ball()


class DribbleLeft(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.duration_forward = parameters.get("duration_forward", 0.5)
        self.duration_turn = parameters.get("angle_turn", 70.0)
        self.aligned = False

        # Store current heading at init
        self.initial_heading = self.blackboard.world_model.get_current_position()[2]
        self.blackboard.node.get_logger().info(f"Initial heading: {np.rad2deg(self.initial_heading):.2f} degrees")

    def perform(self, reevaluate=False):
        """
        Dribbles the ball left. Fully dead reckoning, so it just publishes the corresponding velocities for a certain duration.

        :param reevaluate:
        :return:
        """

        if self.aligned and (self.blackboard.node.get_clock().now() - self.start_time).nanoseconds > self.duration_forward * 1e9:
            self.pop()
            return

        # Stop the pathfinding if it is running for some reason
        self.blackboard.pathfinding.cancel_goal()


        ball_pos = self.blackboard.world_model.get_ball_position_uv()

        if not self.aligned:
            heading = self.blackboard.world_model.get_current_position()[2]
            # Absolute minimal angle difference
            heading_diff = abs(np.arctan2(np.sin(heading - self.initial_heading), np.cos(heading - self.initial_heading)))
            # Log the heading difference for debugging
            self.blackboard.node.get_logger().info(f"Heading difference: {np.rad2deg(heading_diff):.2f} degrees")
            if heading_diff > np.deg2rad(self.duration_turn):
                self.aligned = True
                self.start_time = self.blackboard.node.get_clock().now()
                self.blackboard.node.get_logger().info("Aligned with the target heading.")

        # Publish the velocities
        cmd_vel = Twist()
        if not self.aligned:

            target_dist = 0.25

            x_speed = (ball_pos[0] - target_dist) * 1.0
            rotation_to_face_ball = 0.8 + np.sign(ball_pos[1]) * 1.0
            cmd_vel.linear.x = np.clip(x_speed, -1.0, 1.0)
            cmd_vel.linear.y = -0.3
            cmd_vel.angular.z = np.clip(rotation_to_face_ball, -1.0, 1.0)
        else:
            self.blackboard.node.get_logger().info("Go forward while dribbling left.")
            cmd_vel.linear.x = 1.0
            cmd_vel.linear.y  = np.clip(ball_pos[1] * 3.0, -0.5, 0.5)
            cmd_vel.angular.z = 0.0
        self.blackboard.pathfinding.direct_cmd_vel_pub.publish(cmd_vel)

    def on_pop(self):
        self.blackboard.world_model.forget_ball()
