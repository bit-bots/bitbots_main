from io import BytesIO

import rospy
from std_msgs.msg import Int64

from bitbots_quintic_walk.py_quintic_walk import PyWalkWrapper, init_ros, spin_once
from bitbots_msgs.msg import JointCommand, FootPressure
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class PyWalk(object):
    def __init__(self, namespace=""):
        # make namespace end with a /
        if namespace != "" and namespace[-1] != '/':
            namespace = namespace + "/"
        init_ros(namespace)
        self.py_walk_wrapper = PyWalkWrapper(namespace)

    def spin_ros(self):
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
        self.py_walk_wrapper.reset()

    def special_reset(self, state: String, phase: float, cmd_vel_msg: Twist, reset_odometry: bool):
        state_dict = {"PAUSED": 0, "WALKING": 1, "IDLE": 2, "START_MOVEMENT": 3, "STOP_MOVEMENT": 4, "START_STEP": 5,
                      "STOP_STEP": 6, "KICK": 7}
        self.py_walk_wrapper.special_reset(state_dict[state], phase, self._to_cpp(cmd_vel_msg), reset_odometry)

    def step(self, dt: float, cmdvel_msg: Twist, imu_msg, jointstate_msg, pressure_left, pressure_right,
             cartesian_result=False):
        if dt == 0.0:
            # preventing weird spline interpolation errors on edge case
            dt = 0.001
        stepi = self.py_walk_wrapper.step(
            dt,
            self._to_cpp(cmdvel_msg),
            self._to_cpp(imu_msg),
            self._to_cpp(jointstate_msg),
            self._to_cpp(pressure_left),
            self._to_cpp(pressure_right),
            not cartesian_result)

        if cartesian_result:
            return self.get_left_foot_pose(), self.get_right_foot_pose()
        else:
            result = self._from_cpp(stepi, JointCommand)
            return result

    def get_left_foot_pose(self):
        foot_pose = self.py_walk_wrapper.get_left_foot_pose()
        result = self._from_cpp(foot_pose, Pose)
        return result

    def get_right_foot_pose(self):
        foot_pose = self.py_walk_wrapper.get_left_foot_pose()
        result = self._from_cpp(foot_pose, Pose)
        return result

    def set_engine_dyn_reconf(self, param_dict):
        self.py_walk_wrapper.set_engine_dyn_reconf(param_dict)

    def set_node_dyn_reconf(self, param_dict):
        self.py_walk_wrapper.set_node_dyn_reconf(param_dict)

    def get_phase(self):
        return self.py_walk_wrapper.get_phase()

    def get_freq(self):
        return self.py_walk_wrapper.get_freq()

    def get_odom(self):
        odom = self.py_walk_wrapper.get_odom()
        result = self._from_cpp(odom, Odometry)
        return result
