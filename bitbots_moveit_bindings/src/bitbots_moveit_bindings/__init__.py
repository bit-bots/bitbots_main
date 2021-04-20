from moveit_msgs.srv import GetPositionIKResponse, GetPositionIKRequest
from moveit_msgs.srv import GetPositionFKResponse, GetPositionFKRequest
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init, roscpp_shutdown
from io import BytesIO
from bitbots_moveit_bindings import bitbots_moveit_bindings

def to_cpp(msg):
    """Return a serialized string from a ROS message

    Parameters
    ----------
    - msg: a ROS message instance.
    """
    buf = BytesIO()
    msg.serialize(buf)
    value = buf.getvalue()
    return value


def from_cpp(str_msg, cls):
    """Return a ROS message from a serialized string

    Parameters
    ----------
    - str_msg: str, serialized message
    - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
    """
    msg = cls()
    result = msg.deserialize(str_msg)
    return result


class _RosInitializer:
    def __init__(self):
        roscpp_init('bitbots_moveit_bindings', [])

    def __del__(self):
        roscpp_shutdown()


# The ros initializer is used to automatically setup the ros structure on the
# first call to get_position_{ik,fk} and delete it when the module is closed.
_ros_initializer = None

def get_position_ik(request: GetPositionIKRequest, approximate=False):
    global _ros_initializer
    if _ros_initializer is None:
        _ros_initializer = _RosInitializer()
    request_str = to_cpp(request)
    result_str = bitbots_moveit_bindings.getPositionIK(request_str, approximate)
    return from_cpp(result_str, GetPositionIKResponse)


def get_position_fk(request: GetPositionFKRequest):
    global _ros_initializer
    if _ros_initializer is None:
        _ros_initializer = _RosInitializer()
    request_str = to_cpp(request)
    result_str = bitbots_moveit_bindings.getPositionFK(request_str)
    return from_cpp(result_str, GetPositionFKResponse)
