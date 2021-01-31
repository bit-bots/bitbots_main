from io import BytesIO

from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init, roscpp_shutdown
from sensor_msgs.msg import JointState
from bitbots_dynamic_kick.py_dynamic_kick import PyKickWrapper
from bitbots_msgs.msg import JointCommand, KickGoal


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


class PyKick():
    def __init__(self, namespace=""):
        roscpp_init('py_kick', [])
        # make namespace end with a /
        if namespace != "" and namespace[-1] != '/':
            namespace = namespace + "/"
        self.py_kick_wrapper = PyKickWrapper(namespace)

    def __del__(self):
        roscpp_shutdown()

    def init(self, msg: KickGoal):
        return self.py_kick_wrapper.init(to_cpp(msg))

    def step(self, dt: float, joint_state: JointState):
        if dt == 0.0:
            # preventing weird spline interpolation errors on edge case
            dt = 0.001
        step = self.py_kick_wrapper.step(dt, to_cpp(joint_state))
        return from_cpp(step, JointCommand)

    def get_progress(self):
        return self.py_kick_wrapper.get_progress()

    def set_params(self, params_dict):
        self.py_kick_wrapper.set_params(params_dict)
