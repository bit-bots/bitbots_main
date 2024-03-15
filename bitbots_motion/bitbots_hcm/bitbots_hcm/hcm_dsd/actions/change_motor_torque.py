from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement
from bitbots_msgs.msg import JointTorque


class SetTorque(AbstractHCMActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.stiff = parameters.get("stiff", True)

    def perform(self, reevaluate=False):
        if self.blackboard.current_joint_state is None:
            self.blackboard.node.get_logger().warning(
                "Cannot set joint stiffness for teaching mode because no joint states where received!"
            )
            return self.pop()

        self.blackboard.torque_publisher.publish(
            JointTorque(
                joint_names=self.blackboard.current_joint_state.name,
                on=[self.stiff] * len(self.blackboard.current_joint_state.name),
            )
        )
        return self.pop()
