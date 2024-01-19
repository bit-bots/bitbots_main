from bio_ik_msgs.msg import IKRequest
from bio_ik_msgs.srv import GetIK
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rcl_interfaces.msg import Parameter
from rclpy.serialization import deserialize_message, serialize_message

from bitbots_moveit_bindings.libbitbots_moveit_bindings import BitbotsMoveitBindings

# this state is only used for IK and FK calls and does not listen to current joint_states
ik_fk_state = None
# this is only used for collision states. it updates with joint states
collision_state = None


def set_moveit_parameters(parameters: [Parameter], type):
    """
    Allows to set the parameters manually instead of copying them from the move_group node.
    :param parameters: List of Parameter messages
    :return:
    """
    global ik_fk_state, collision_state
    serialized_parameters = []
    for parameter in parameters:
        serialized_parameters.append(serialize_message(parameter))
    if type == "ikfk":
        ik_fk_state = BitbotsMoveitBindings("moveit_bindings_ik", serialized_parameters)
    elif type == "collision":
        collision_state = BitbotsMoveitBindings("moveit_bindings_collision", serialized_parameters)
    else:
        print("Type for bitbots_moveit_bindings state not correctly defined")
        exit(1)


def get_position_ik(request: GetPositionIK.Request, approximate=False):
    global ik_fk_state
    if ik_fk_state is None:
        ik_fk_state = BitbotsMoveitBindings("moveit_bindings_ik", [])
    request_str = serialize_message(request)
    result_str = ik_fk_state.getPositionIK(request_str, approximate)
    return deserialize_message(result_str, GetPositionIK.Response)


def get_position_fk(request: GetPositionFK.Request):
    global ik_fk_state
    if ik_fk_state is None:
        ik_fk_state = BitbotsMoveitBindings("moveit_bindings_ik", [])
    result_str = ik_fk_state.getPositionFK(serialize_message(request))
    return deserialize_message(result_str, GetPositionFK.Response)


def get_bioik_ik(request: IKRequest) -> GetIK.Response:
    global ik_fk_state
    if ik_fk_state is None:
        ik_fk_state = BitbotsMoveitBindings("moveit_bindings_ik", [])
    result_str = ik_fk_state.getBioIKIK(serialize_message(request))
    return deserialize_message(result_str, GetIK.Response)


def check_collision(joint_state):
    global collision_state
    if collision_state is None:
        collision_state = BitbotsMoveitBindings("moveit_bindings_collision", [])
    collision_state.set_joint_states(serialize_message(joint_state))
    collision = collision_state.check_collision()
    return collision
