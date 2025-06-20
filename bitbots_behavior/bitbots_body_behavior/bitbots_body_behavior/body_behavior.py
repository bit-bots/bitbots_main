import os

import rclpy
from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_tf_buffer import Buffer
from dynamic_stack_decider.dsd import DSD
from game_controller_hl_interfaces.msg import GameState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from soccer_vision_3d_msgs.msg import RobotArray

from bitbots_body_behavior import behavior_dsd
from bitbots_msgs.msg import RobotControlState, TeamData


class BodyDSD:
    def __init__(self, node: Node):
        self.counter = 0
        self.step_running = False
        self.node = node

        self.tf_buffer = Buffer(Duration(seconds=30), node)

        blackboard = BodyBlackboard(node, self.tf_buffer)
        self.dsd = DSD(blackboard, "debug/dsd/body_behavior", node)  # TODO: use config

        self.dsd.register_actions(behavior_dsd.actions.__path__[0])
        self.dsd.register_decisions(behavior_dsd.decisions.__path__[0])

        dsd_file: str = node.get_parameter("dsd_file").value
        self.dsd.load_behavior(os.path.join(behavior_dsd.__path__[0], dsd_file))

        node.create_subscription(
            PoseWithCovarianceStamped,
            "ball_position_relative_filtered",
            blackboard.world_model.ball_filtered_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        node.create_subscription(
            GameState,
            "gamestate",
            blackboard.gamestate.gamestate_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        node.create_subscription(
            TeamData,
            "team_data",
            blackboard.team_data.team_data_callback,
            qos_profile=10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        node.create_subscription(
            RobotArray,
            "robots_relative_filtered",
            blackboard.costmap.robot_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        node.create_subscription(
            RobotControlState,
            "robot_state",
            blackboard.misc.robot_state_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        node.create_subscription(
            Twist,
            "cmd_vel",
            blackboard.pathfinding.cmd_vel_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def loop(self):
        try:
            self.dsd.update()
            blackboard: BodyBlackboard = self.dsd.blackboard
            blackboard.team_data.publish_strategy()
            blackboard.team_data.publish_time_to_ball()
            self.counter = (self.counter + 1) % blackboard.config["time_to_ball_divider"]
            if self.counter == 0:
                blackboard.pathfinding.calculate_time_to_ball()
            blackboard.clear_cache()
        except Exception as e:
            import traceback

            traceback.print_exc()
            self.node.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=None)
    node = Node("body_behavior", automatically_declare_parameters_from_overrides=True)
    body_dsd = BodyDSD(node)
    node.create_timer(1 / 60.0, body_dsd.loop, callback_group=MutuallyExclusiveCallbackGroup(), clock=node.get_clock())

    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
