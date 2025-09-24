# Setting up runtime type checking for this package
try:
    from beartype.claw import beartype_this_package
    beartype_this_package()
except ImportError:
    # beartype not available, skip type checking
    pass

from rclpy.node import Node

from bitbots_vision.vision_parameters import bitbots_vision as parameters


class NodeWithConfig(Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.param_listener = parameters.ParamListener(self)
        self.config = self.param_listener.get_params()
