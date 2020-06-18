from io import BytesIO

import rospy
from std_msgs.msg import Int64

from bitbots_quintic_walk.py_quintic_walk import PyWalkWrapper
from bitbots_msgs.msg import JointCommand
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState


class PyWalk(object):
    def __init__(self):
        self.py_walk_wrapper = PyWalkWrapper()

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
        self.py_walk_wrapper.reset()

    def step(self, dt: float, cmdvel_msg: Twist, imu_msg, jointstate_msg):
        stepi = self.py_walk_wrapper.step(
            dt,
            self._to_cpp(cmdvel_msg),
            self._to_cpp(imu_msg),
            self._to_cpp(jointstate_msg))

        result = self._from_cpp(
            stepi,
            JointCommand
        )

        return result
