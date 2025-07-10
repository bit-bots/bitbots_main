from typing import Callable

import rclpy
from geometry_msgs.msg import PointStamped, WrenchStamped
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.time import Time

from bitbots_msgs.msg import FootPressure


class FootPressureProxy(Node):
    """
    A Node which converts published `WrenchStamped` messages of all foot `force_sensors_1d` sensors of the hlvs_player
    into a combined `FootPressure` message for each foot, which is used e.g. in walking for the phase_reset.
    This is based on the `bitbots_webots_sim/webots_robot_controller.py`, which is used for local webots simulation and
    does not actually use the `bitbots_ros_control/pressure_converter.cpp` node, which would be used on the real robot.
    """

    def __init__(self):
        super().__init__("foot_pressure_proxy")
        self.logger = self.get_logger()

        self.declare_parameter("without_pressure_converter", True)
        self.without_pressure_converter = (
            self.get_parameter("without_pressure_converter").get_parameter_value().bool_value
        )
        self.logger.info(f"Running with{'out' if self.without_pressure_converter else ''} pressure converter.")
        if self.without_pressure_converter:
            self.logger.info("Will publish raw, filtered foot pressure and center of pressure (CoP) messages.")
        else:
            self.logger.info("Will only publish raw foot pressure messages.")

        self.pressure_sensor_names = ["llb", "llf", "lrf", "lrb", "rlb", "rlf", "rrf", "rrb"]
        self.foot_pressure_mappings = {
            "lb": "left_back",
            "lf": "left_front",
            "rf": "right_front",
            "rb": "right_back",
        }
        self.received_left_wrench_messages = dict.fromkeys(self.foot_pressure_mappings.values(), WrenchStamped())
        self.received_right_wrench_messages = dict.fromkeys(self.foot_pressure_mappings.values(), WrenchStamped())

        self.l_sole_frame = "l_sole"
        self.left_pressure = FootPressure()
        self.left_pressure.header.frame_id = self.l_sole_frame

        self.r_sole_frame = "r_sole"
        self.right_pressure = FootPressure()
        self.right_pressure.header.frame_id = self.r_sole_frame

        self.create_wrench_pressure_subscriptions()
        self.create_foot_pressure_publishers()

    def create_wrench_pressure_subscriptions(self):
        for name in self.pressure_sensor_names:
            self.create_subscription(
                WrenchStamped, f"{name}/data", lambda msg, name=name: self.listener_callback(name, msg), 10
            )

    def create_foot_pressure_publishers(self):
        self.pub_pressure_left = self.create_publisher(FootPressure, "foot_pressure_left/raw", 1)
        self.pub_pressure_right = self.create_publisher(FootPressure, "foot_pressure_right/raw", 1)
        if self.without_pressure_converter:
            self.pub_pressure_left_filtered = self.create_publisher(FootPressure, "foot_pressure_left/filtered", 1)
            self.pub_pressure_right_filtered = self.create_publisher(FootPressure, "foot_pressure_right/filtered", 1)
            self.pub_cop_left = self.create_publisher(PointStamped, "cop_l", 1)
            self.pub_cop_right = self.create_publisher(PointStamped, "cop_r", 1)

    def listener_callback(self, pressure_sensor_name: str, msg: WrenchStamped):
        msg_dict, sensor_location, publish_fn = self.get_foot_properties(pressure_sensor_name)
        msg_dict[sensor_location] = msg

        all_foot_timestamps_equal = all(
            Time.from_msg(wrench_msg.header.stamp) == Time.from_msg(msg.header.stamp)
            for wrench_msg in msg_dict.values()
        )

        # We only want to publish a foots FootPressure message if the received wrench messages
        # all belong to the current hlvs_player step, i.e. they have the same timestamp.
        if all_foot_timestamps_equal:
            publish_fn()

    def get_foot_properties(self, pressure_sensor_name: str) -> tuple[dict[str, WrenchStamped], str, Callable]:
        foot, sensor_location_shorthand = pressure_sensor_name[0], pressure_sensor_name[1:]

        if foot == "l":
            msg_dict = self.received_left_wrench_messages
            publish_fn = self.publish_left_pressure
        else:
            msg_dict = self.received_right_wrench_messages
            publish_fn = self.publish_right_pressure

        return (msg_dict, self.foot_pressure_mappings[sensor_location_shorthand], publish_fn)

    def create_foot_pressure_msg(
        self, frame: str, latest_stamp: Time, wrench_msgs: dict[str, WrenchStamped]
    ) -> FootPressure:
        pressure_msg = FootPressure()
        pressure_msg.header.frame_id = frame
        pressure_msg.header.stamp = latest_stamp.to_msg()
        for sensor_location, wrench_msg in wrench_msgs.items():
            setattr(pressure_msg, sensor_location, wrench_msg.wrench.force.z)

        return pressure_msg

    def create_foot_center_of_pressure_msg(self, latest_stamp: Time, pressure_msg: FootPressure) -> PointStamped:
        """
        Copied and adapted from the `bitbots_webots_sim/webots_robot_controller.py` file.
        """
        # compute center of pressures of the feet
        pos_x = 0.085
        pos_y = 0.045
        # we can take a very small threshold, since simulation gives more accurate values than reality
        threshold = 1

        cop = PointStamped()
        cop.header.frame_id = pressure_msg.header.frame_id
        cop.header.stamp = latest_stamp.to_msg()
        sum = pressure_msg.left_back + pressure_msg.left_front + pressure_msg.right_front + pressure_msg.right_back
        if sum > threshold:
            cop.point.x = (
                (pressure_msg.left_front + pressure_msg.right_front - pressure_msg.left_back - pressure_msg.right_back)
                * pos_x
                / sum
            )
            cop.point.x = max(min(cop.point.x, pos_x), -pos_x)
            cop.point.y = (
                (pressure_msg.left_front + pressure_msg.left_back - pressure_msg.right_front - pressure_msg.right_back)
                * pos_y
                / sum
            )
            cop.point.y = max(min(cop.point.x, pos_y), -pos_y)
        else:
            cop.point.x = 0.0
            cop.point.y = 0.0

        return cop

    def publish_left_pressure(self):
        latest_stamp = max(Time.from_msg(msg.header.stamp) for msg in self.received_left_wrench_messages.values())
        left = self.create_foot_pressure_msg(self.l_sole_frame, latest_stamp, self.received_left_wrench_messages)
        self.pub_pressure_left.publish(left)

        if self.without_pressure_converter:
            cop_l = self.create_foot_center_of_pressure_msg(latest_stamp, left)
            self.pub_cop_left.publish(cop_l)
            self.pub_pressure_left_filtered.publish(left)

    def publish_right_pressure(self):
        latest_stamp = max(Time.from_msg(msg.header.stamp) for msg in self.received_left_wrench_messages.values())
        right = self.create_foot_pressure_msg(self.r_sole_frame, latest_stamp, self.received_right_wrench_messages)
        self.pub_pressure_right.publish(right)

        if self.without_pressure_converter:
            cop_r = self.create_foot_center_of_pressure_msg(latest_stamp, right)
            self.pub_cop_right.publish(cop_r)
            self.pub_pressure_right_filtered.publish(right)


def main(args=None):
    rclpy.init(args=args)
    foot_pressure_proxy = FootPressureProxy()
    executor = EventsExecutor()
    executor.add_node(foot_pressure_proxy)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    foot_pressure_proxy.destroy_node()
