from io import BytesIO

import rospy
from std_msgs.msg import Int64

from bitbots_dynup.py_dynup import PyDynupWrapper, init_ros, spin_once
from bitbots_dynup.msg import DynupPoses
from bitbots_msgs.msg import JointCommand, FootPressure
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class PyDynup(object):
    def __init__(self, namespace=""):
        if namespace != "" and namespace[-1] != '/':
            namespace = namespace + '/'
        init_ros(namespace)
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

    def step(self, dt: float, imu_msg, jointstate_msg):
        if dt == 0.0:
            dt = 0.001
        stepi = self.py_dynup_wrapper.step(dt, self._to_cpp(imu_msg), self._to_cpp(jointstate_msg))

        result = self._from_cpp(stepi, JointCommand)

        return result

    def step_open_loop(self, dt: float):
        if dt == 0.0:
            dt = 0.001
        stepi = self.py_dynup_wrapper.step_open_loop(dt)

        result = self._from_cpp(stepi, JointCommand)

        return result

    def get_poses(self):
        poses = self.py_dynup_wrapper.get_poses()
        result = self._from_cpp(poses, DynupPoses)
        return result

    def set_node_dyn_reconf(self, param_dict):
        self.py_dynup_wrapper.set_node_dyn_reconf(param_dict)

    def get_direction(self):
        return self.py_dynup_wrapper.get_direction()



