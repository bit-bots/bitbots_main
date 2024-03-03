#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from bitbots_msgs.msg import JointCommand, JointTorque


class TeachingNode(Node):
    def __init__(self):
        super().__init__("teaching_node")
        self.is_button_pressed = False
        self.timer = self.create_timer(0.1, self.publish_command)
        self.get_logger().info("Teaching Node has been started")
        self.joint_states: JointState | None = None
        self.default_velocity = 5.0
        self.default_accelerations = -1.0
        self.default_max_currents = -1.0

        self.joint_command_publisher = self.create_publisher(JointCommand, "DynamixelController/command", 10)
        self.torque_publisher = self.create_publisher(JointTorque, "ros_control/set_torque_individual", 10)
        self.subscription = self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

        self.activation_service = self.create_service(SetBool, "set_teaching_mode", self.activation_service_callback)

    def activation_service_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if self.joint_states is None:
            self.get_logger().warning(
                "Cannot set joint stiffness for teaching mode because no joint states where recived!"
            )
            response.success = False
            return response

        self.torque_publisher.publish(
            JointTorque(joint_names=self.joint_states.name, on=[not request.data] * len(self.joint_states.name))
        )
        if not request.data:
            self.joint_command_publisher.publish(
                JointCommand(
                    joint_names=self.joint_states.name,
                    positions=self.joint_states.position,
                    accelerations=[self.default_accelerations] * len(self.joint_states.name),
                    max_currents=[self.default_max_currents] * len(self.joint_states.name),
                    velocities=[self.default_velocity] * len(self.joint_states.name),
                )
            )
        response.success = True
        return response

    def joint_states_callback(self, msg: JointState):
        self.joint_states = msg


if __name__ == "__main__":
    rclpy.init()
    node = TeachingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

# where publish
# were subscribe
# set torque benutzen
