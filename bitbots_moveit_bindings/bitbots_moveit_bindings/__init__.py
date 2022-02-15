from moveit_msgs.srv import GetPositionIKResponse, GetPositionIKRequest
from moveit_msgs.srv import GetPositionFKResponse, GetPositionFKRequest
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init, roscpp_shutdown
from io import BytesIO
from bitbots_moveit_bindings.bitbots_moveit_bindings import *
from rclpy.serialization import serialize_message, deserialize_message


class _RosInitializer:
    def __init__(self):
        self.node = BitbotsMoveitBindings()

    def __del__(self):
        self.node.shutdown()


# The ros initializer is used to automatically setup the ros structure on the
# first call to get_position_{ik,fk} and delete it when the module is closed.
_ros_initializer = None


def get_position_ik(request: GetPositionIKRequest, approximate=False):
    global _ros_initializer
    if _ros_initializer is None:
        _ros_initializer = _RosInitializer()
    request_str = serialize_message(request)
    result_str = _ros_initializer.getPositionIK(request_str, approximate)
    return deserialize_message(result_str, GetPositionIKResponse)


def get_position_fk(request: GetPositionFKRequest):
    global _ros_initializer
    if _ros_initializer is None:
        _ros_initializer = _RosInitializer()
    request_str = serialize_message(request)
    result_str = _ros_initializer.getPositionFK(request_str)
    return deserialize_message(result_str, GetPositionFKResponse)
