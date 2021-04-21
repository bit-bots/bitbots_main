from io import BytesIO

from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init, roscpp_shutdown
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
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


class PyKick:
    """
    This class is a wrapper around the dynamic kick. When an instance is created, a roscore must be running and
    the robot description must be loaded to the parameter server. To use the wrapper, call set_params to set a
    dictionary of parameters. A list of parameters can be found in the dynamic reconfigure file, the pid controllers are
    not supported yet. Then, set the kick goal by calling init. After that, repeatedly call step with the passed time
    and the current motor positions of the robot. It will return the new motor commands.
    """
    def __init__(self, namespace=""):
        """
        Initialize the kick, needs a roscore and the robot description

        :param namespace: The namespace where the kick node is launched
        """
        roscpp_init('py_kick', [])
        # make namespace end with a /
        if namespace != "" and namespace[-1] != '/':
            namespace = namespace + "/"
        self.py_kick_wrapper = PyKickWrapper(namespace)

    def __del__(self):
        roscpp_shutdown()

    def set_goal(self, msg: KickGoal, joint_state: JointState) -> bool:
        """
        Set a goal for the kick.

        :param msg: The goal, instance of bitbots_msgs/KickGoal
        :param joint_state: The current motor positions of the robot
        :return: whether the goal was set successfully
        """
        return self.py_kick_wrapper.set_goal(to_cpp(msg), to_cpp(joint_state))

    def step(self, dt: float, joint_state: JointState) -> JointCommand:
        """
        Perform a step of the kick engine, must be repeatedly called to perform the full kick.

        :param dt: the time passed since the last call of step (in seconds)
        :param joint_state: the current motor positions of the robot
        :return: Returns the joint command for the next position of the kick. Empty JointCommand if finished.
        """
        if dt == 0.0:
            # preventing weird spline interpolation errors on edge case
            dt = 0.001
        elif dt > 1:
            print('dt is very large, maybe forgot to reset?')
        step = self.py_kick_wrapper.step(dt, to_cpp(joint_state))
        return from_cpp(step, JointCommand)

    def get_progress(self) -> float:
        """Returns the progress of the kick, between 0 and 1 where 1 is finished."""
        return self.py_kick_wrapper.get_progress()

    def set_params(self, params_dict):
        """
        Set the (dynamic reconfigurable) parameters of the kick.
        :param params_dict: dict where the key is the name of the parameter (str). Does not have to contain all params.
        """
        self.py_kick_wrapper.set_params(params_dict)

    def get_trunk_pose(self) -> Pose:
        """Returns the current pose of the trunk relative to the support foot"""
        return from_cpp(self.py_kick_wrapper.get_trunk_pose(), Pose)
