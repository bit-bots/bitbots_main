from moveit_msgs.srv import GetPositionIK, GetPositionIK
from moveit_msgs.srv import GetPositionFK, GetPositionFK
from bitbots_moveit_bindings.libbitbots_moveit_bindings import BitbotsMoveitBindings
from rclpy.serialization import serialize_message, deserialize_message


bindings = None


def get_position_ik(request: GetPositionIK.Request, approximate=False):
    global bindings
    if bindings is None:
        bindings = BitbotsMoveitBindings()
    request_str = serialize_message(request)
    result_str = bindings.getPositionIK(request_str, approximate)
    return deserialize_message(result_str, GetPositionIK.Response)


def get_position_fk(request: GetPositionFK.Request):
    global bindings
    if bindings is None:
        bindings = BitbotsMoveitBindings()
    result_str = bindings.getPositionFK(request)
    return deserialize_message(result_str, GetPositionFK.Response)
