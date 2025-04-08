# Setting up runtime type checking for this package
from beartype.claw import beartype_this_package
from rclpy.node import Node

from bitbots_path_planning.path_planning_parameters import bitbots_path_planning as parameters

beartype_this_package()


class NodeWithConfig(Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.param_listener = parameters.ParamListener(self)
        self.config = self.param_listener.get_params()
