from enum import Enum, unique
import math
import time

import rclpy
from game_controller_hl_interfaces.msg import GameState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist  # noqa
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Empty, Bool  # noqa
from transitions.extensions import GraphMachine
import tf2_ros as tf2
from tf2_ros import Buffer, TransformListener

from transforms3d.euler import euler2quat, quat2euler

from bitbots_msgs.msg import HeadMode  # noqa


# Hier stehen alle states, die eure StateMachine benutzt
@unique
class States(Enum):
    INITIAL = 0
    READY_OUR_KICKOFF = 1
    READY_THEIR_KICKOFF = 2
    SET = 3
    GO_TO_BALL = 4
    KICK_LEFT = 5
    KICK_RIGHT = 6
    SEARCH_BALL = 7


class StateMachine(Node):
    def __init__(self):
        super().__init__("behavior_praktikum")

        self.state: States

        # Hier erstellt ihr eure StateMachine. Dabei wird in "states" das enum von oben angegeben und in "initial" der anfangs state gesetzt.
        self.machine = GraphMachine(
            model=self, states=States, initial=States.INITIAL, title="Behavior", show_conditions=True
        )

        # Wenn ihr eine Transition erstellt, müsst ihr einen Trigger, einen oder mehrere source states angeben, aus denen dann in den dest state gewechselt werden kann.
        # Hier kann aus jedem anderen state in initial gewechselt werden, wenn die Triggerfunktion initial() aufgerufen wird.
        self.machine.add_transition(trigger="initial", source="*", dest=States.INITIAL)
        self.machine.add_transition(trigger="ready", source="*", dest=States.READY_OUR_KICKOFF, conditions=self.decision_kickoff)
        self.machine.add_transition(trigger="ready", source="*", dest=States.READY_THEIR_KICKOFF)
        self.machine.add_transition(trigger="set", source="*", dest=States.SET)
        self.machine.add_transition(trigger="play", source=States.GO_TO_BALL, dest=States.KICK_LEFT, conditions=[self.decision_ball_close, self.decision_ball_on_left_side])
        self.machine.add_transition(trigger="play", source=States.GO_TO_BALL, dest=States.KICK_RIGHT, conditions=self.decision_ball_close)
        self.machine.add_transition(trigger="play", source="*", dest=States.GO_TO_BALL, conditions=self.decision_ball_seen)
        self.machine.add_transition(trigger="play", source="*", dest=States.SEARCH_BALL)

        # TODO add more transitions here

        # Render the state machine as a graph, png can be found in colcon_ws folder
        self.get_graph().draw("behavior.png", prog="dot")

        # Create publisher
        self.velocity_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cancel_navigation_publisher_ = self.create_publisher(Empty, "pathfinding/cancel", 10)
        self.navigation_goal_publisher_ = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.head_mode_publisher_ = self.create_publisher(HeadMode, "head_mode", 10)
        self.kick_publisher_ = self.create_publisher(Bool, "kick", 10)

        # Create subscriber
        self.gamestate_subscription = self.create_subscription(GameState, "gamestate", self.gamestate_callback, 10)
        self.ball_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/ball_position_relative_filtered", self.ball_callback, 10
        )

        # Create tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Blackboard
        self.has_kickoff: bool = False
        self.ball_position_x: float = 0.0
        self.ball_position_y: float = 0.0
        self.ball_std: float = 0.0
        self.goal_position = (4.5, 0.0)
        self.goal_width = 2.6
        self.goal_safety_border = 0.3
        self.kick_x_offset = -0.2
        self.kick_y_offset = 0.05

    def ball_callback(self, msg: PoseWithCovarianceStamped):
        self.ball_position_x = msg.pose.pose.position.x
        self.ball_position_y = msg.pose.pose.position.y
        self.ball_std = msg.pose.covariance[0]

    def gamestate_callback(self, msg: GameState):
        # Tht needs to be first, otherwise the condition for the transition will use the old value
        self.has_kickoff = msg.has_kick_off

        match msg.game_state:
            case GameState.GAMESTATE_INITIAL:
                self.initial()
            case GameState.GAMESTATE_READY:
                self.ready()
            case GameState.GAMESTATE_SET:
                self.set()
            case GameState.GAMESTATE_PLAYING:
                self.play()

    def get_my_pose(self) -> tuple[float, float, float]:
        # Get the robot pose in the map frame
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())

            # Rotation
            yaw = quat2euler(
                [
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                ]
            )[2]

            return transform.transform.translation.x, transform.transform.translation.y, yaw
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            self.get_logger().warn("Could not get transform from map to base_footprint")
            return 0.0, 0.0, 0.0

    def get_ball_distance(self) -> float:
        # Get the distance to the ball
        my_pose = self.get_my_pose()
        return math.hypot(
            my_pose[0] - self.ball_position_x,
            my_pose[1] - self.ball_position_y,
        )

    def decision_kickoff(self):
        return self.has_kickoff

    def decision_ball_seen(self):
        return self.ball_std < 2.0

    def decision_ball_close(self):
        return self.get_ball_distance() < 0.3

    def decision_ball_on_left_side(self):
        my_pose = self.get_my_pose()
        ball_pose = (self.ball_position_x, self.ball_position_y)

        # Calculate the angle between the robot and the ball
        angle = math.atan2(ball_pose[1] - my_pose[1], ball_pose[0] - my_pose[0])

        # Compute minimum angle difference
        angle_diff = math.atan2(math.sin(angle - my_pose[2]), math.cos(angle - my_pose[2]))

        # Check if the ball is on the left side of the robot
        return angle_diff > 0.0

    def action_stand(self):
        # Cancel all path navigation
        self.cancel_navigation_publisher_.publish(Empty())
        # Stop the robots walking
        velocity = Twist()
        velocity.angular.x = -1.0  # stops the robot
        self.velocity_publisher_.publish(velocity)

    def action_go_to_pose(self, x, y, theta):
        # Convert euler angles to quaternion
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        w, x, y, z = euler2quat(0.0, 0.0, theta)
        goal_pose.pose.orientation.x = x
        goal_pose.pose.orientation.y = y
        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = w
        self.navigation_goal_publisher_.publish(goal_pose)

    def action_set_head_mode(self, mode):
        head_mode = HeadMode()
        head_mode.head_mode = mode
        self.head_mode_publisher_.publish(head_mode)

    def action_go_to_ball_simple(self):
        self.action_go_to_pose(self.ball_position_x + self.kick_x_offset, self.ball_position_y + self.kick_y_offset, 0.0)

    def action_go_to_ball(self):
        ball_position = (self.ball_position_x, self.ball_position_y)
        goal_position = self.goal_position

        # If the ball is the center of the field we can just play forward to avoid weird behavior near the goal / playing always to the exact center
        if abs(ball_position[1]) < self.goal_width / 2 - self.goal_safety_border:
            self.action_go_to_ball_simple()
        else:
            # Vector from ball to goal
            ball_to_goal_vector = (goal_position[0] - ball_position[0], goal_position[1] - ball_position[1])

            # Normalize the vector
            length = math.hypot(ball_to_goal_vector[0], ball_to_goal_vector[1])
            ball_to_goal_normalized = (
                ball_to_goal_vector[0] / length,
                ball_to_goal_vector[1] / length,
            )

            # Calc the goal position
            # The goal position is slightly offset away from the goal, so we stand behind the ball
            # We also offset it slightly to the side to avoid the ball between both feet
            target_position = (
                ball_position[0]
                    + ball_to_goal_normalized[0] * self.kick_x_offset
                    + self.rotate_vector_by_90_degrees(ball_to_goal_normalized)[0] * self.kick_y_offset,
                ball_position[1]
                    + ball_to_goal_normalized[1] * self.kick_x_offset 
                    + self.rotate_vector_by_90_degrees(ball_to_goal_normalized)[1] * self.kick_y_offset,
            )

            # Calculate the target angle
            target_angle = math.atan2(ball_to_goal_vector[1], ball_to_goal_vector[0])

            # Set goal pose
            self.action_go_to_pose(target_position[0], target_position[1], target_angle)

    def action_kick(self, right: bool):
        kick = Bool()
        kick.data = right
        self.kick_publisher_.publish(kick)

    def action_turn(self):
        # Cancel all path navigation
        self.cancel_navigation_publisher_.publish(Empty())
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.7
        self.velocity_publisher_.publish(cmd_vel)

    def rotate_vector_by_90_degrees(self, vector):
        # Rotate the vector by 90 degrees
        return -vector[1], vector[0]

    # Hier implementiert ihr, was der Roboter in jedem State tun soll.
    def timer_callback(self):
        self.get_logger().info(f"State: {self.state}")
        match self.state:
            case States.INITIAL | States.SET:
                self.action_stand()
                self.action_set_head_mode(HeadMode.LOOK_FORWARD)
            case States.READY_OUR_KICKOFF:
                self.action_go_to_pose(-0.2, 0.0, 0.0)
                self.action_set_head_mode(HeadMode.SEARCH_FRONT)
            case States.READY_THEIR_KICKOFF:
                self.action_go_to_pose(-1.0, 0.0, 0.0)
                self.action_set_head_mode(HeadMode.SEARCH_FIELD_FEATURES)
            case States.GO_TO_BALL:
                self.action_go_to_ball()
                self.action_set_head_mode(HeadMode.SEARCH_FRONT)
                self.play() # We need to call play() to trigger the next state
            case States.KICK_LEFT:
                self.action_kick(right=False)
                time.sleep(2.0)
                self.play() # We need to call play() to trigger the next state
            case States.KICK_RIGHT:
                self.action_kick(right=True)
                self.play() # We need to call play() to trigger the next state
            case States.SEARCH_BALL:
                self.action_turn()
                self.action_set_head_mode(HeadMode.SEARCH_BALL)
                self.play()  # We need to call play() to trigger the next state
            case state:
                self.get_logger().warn(f"State: Unknown state: {state}")


def main(args=None):
    rclpy.init()
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
    rclpy.shutdown()
