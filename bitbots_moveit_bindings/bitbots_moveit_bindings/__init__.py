from moveit_msgs.srv import GetPositionIK, GetPositionIK
from moveit_msgs.srv import GetPositionFK, GetPositionFK
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


def get_position_ik(request: GetPositionIK.Request, approximate=False):
    global _ros_initializer
    if _ros_initializer is None:
        _ros_initializer = _RosInitializer()
    request_str = serialize_message(request)
    result_str = _ros_initializer.getPositionIK(request_str, approximate)
    return deserialize_message(result_str, GetPositionIK.Response)


def get_position_fk(request: GetPositionFK.Request):
    global _ros_initializer
    if _ros_initializer is None:
        _ros_initializer = _RosInitializer()
    request_str = serialize_message(request)
    result_str = _ros_initializer.getPositionFK(request_str)
    return deserialize_message(result_str, GetPositionFK.Response)
