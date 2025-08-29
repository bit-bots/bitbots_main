from io import BytesIO

from geometry_msgs.msg import PoseArray

from bitbots_dynup.msg import DynupPoses
from bitbots_dynup.py_dynup import PyDynupWrapper, spin_once
from bitbots_msgs.msg import JointCommand


class PyDynup:
    def __init__(self, namespace=""):
        if namespace != "" and namespace[-1] != "/":
            namespace = namespace + "/"
        self.py_dynup_wrapper = PyDynupWrapper(namespace)

    def spin_once(self):
        spin_once()

    def _to_cpp(self, msg):
        """Return a serialized string from a ROS message

        Parameters
        ----------
        - msg: a ROS message instance.
        """
        buf = BytesIO()
        msg.serialize(buf)
        value = buf.getvalue()
        return value

    def _from_cpp(self, str_msg, cls):
        """Return a ROS message from a serialized string

        Parameters
        ----------
        - str_msg: str, serialized message
        - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
        """
        msg = cls()
        result = msg.deserialize(str_msg)
        return result

    def reset(self):
        self.py_dynup_wrapper.reset()

    def special_reset(self, time: float):
        self.py_dynup_wrapper.special_reset(time)

    def set_engine_goal(self, direction):
        self.py_dynup_wrapper.set_engine_goal(direction)

    def step(self, dt: float, imu_msg, joint_state_msg):
        if dt == 0.0:
            dt = 0.001
        step = self.py_dynup_wrapper.step(dt, self._to_cpp(imu_msg), self._to_cpp(joint_state_msg))
        result = self._from_cpp(step, JointCommand)
        return result

    def step_open_loop(self, dt: float):
        if dt == 0.0:
            dt = 0.001
        stepi = self.py_dynup_wrapper.step_open_loop(dt)

        result = self._from_cpp(stepi, PoseArray)
        return result

    def get_poses(self):
        poses = self.py_dynup_wrapper.get_poses()
        result = self._from_cpp(poses, DynupPoses)
        return result

    def set_node_dyn_reconf(self, param_dict):
        self.get_logger().error("python params", param_dict)
        self.py_dynup_wrapper.set_node_dyn_reconf(param_dict)

    def get_direction(self):
        return self.py_dynup_wrapper.get_direction()
