from moveit_msgs.srv import GetPositionFK, GetPositionIK
from bitbots_moveit_bindings.libbitbots_moveit_bindings import BitbotsMoveitBindings
from rclpy.serialization import serialize_message, deserialize_message
from rcl_interfaces.msg import Parameter

bindings = None

def set_moveit_parameters(parameters: [Parameter]):
    """
    Allows to set the parameters manually instead of copying them from the move_group node.
    :param parameters: List of Parameter messages
    :return:
    """
    global bindings
    serialized_parameters = []
    for parameter in parameters:
        serialized_parameters.append(serialize_message(parameter))
    bindings = BitbotsMoveitBindings(serialized_parameters)

def get_position_ik(request: GetPositionIK.Request, approximate=False):
    global bindings
    if bindings is None:
        bindings = BitbotsMoveitBindings([])
    request_str = serialize_message(request)
    result_str = bindings.getPositionIK(request_str, approximate)
    return deserialize_message(result_str, GetPositionIK.Response)


def get_position_fk(request: GetPositionFK.Request):
    global bindings
    if bindings is None:
        bindings = BitbotsMoveitBindings([])
    result_str = bindings.getPositionFK(serialize_message(request))
    return deserialize_message(result_str, GetPositionFK.Response)
