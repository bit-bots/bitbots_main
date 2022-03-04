from ros2param.api import parse_parameter_dict
from std_msgs.msg import Int64

from bitbots_quintic_walk_py.libpy_quintic_walk import PyWalkWrapper, initRos
from bitbots_msgs.msg import JointCommand, FootPressure
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message, deserialize_message
from rcl_interfaces.msg import Parameter

class PyWalk:
    def __init__(self, namespace="", parameters: [Parameter]=[]):
        # make namespace end with a /
        if namespace != "" and namespace[-1] != '/':
            namespace = namespace + "/"
        serialized_parameters = []
        for parameter in parameters:
            serialized_parameters.append(serialize_message(parameter))
            if parameter.value.type == 2:
                print(f"Gave parameter {parameter.name} of integer type. If the code crashes it is maybe because this "
                      f"should be a float. You may need to add an .0 in some yaml file.")
        self.py_walk_wrapper = PyWalkWrapper(namespace, serialized_parameters)

    def spin_ros(self):
        self.py_walk_wrapper.spin_some()

    def reset(self):
        self.py_walk_wrapper.reset()

    def special_reset(self, state: String, phase: float, cmd_vel_msg: Twist, reset_odometry: bool):
        state_dict = {"PAUSED": 0, "WALKING": 1, "IDLE": 2, "START_MOVEMENT": 3, "STOP_MOVEMENT": 4, "START_STEP": 5,
                      "STOP_STEP": 6, "KICK": 7}
        self.py_walk_wrapper.special_reset(state_dict[state], phase, serialize_message(cmd_vel_msg), reset_odometry)

    def step(self, dt: float, cmdvel_msg: Twist, imu_msg, jointstate_msg, pressure_left, pressure_right):
        if dt == 0.0:
            # preventing weird spline interpolation errors on edge case
            dt = 0.001
        stepi = self.py_walk_wrapper.step(
            dt,
            serialize_message(cmdvel_msg),
            serialize_message(imu_msg),
            serialize_message(jointstate_msg),
            serialize_message(pressure_left),
            serialize_message(pressure_right))

        result = deserialize_message(stepi, JointCommand)
        return result

    def step_relative(self, dt: float, step_msg: Twist, imu_msg, jointstate_msg, pressure_left, pressure_right):
        if dt == 0.0:
            # preventing weird spline interpolation errors on edge case
            dt = 0.001
        stepi = self.py_walk_wrapper.step_relative(
            dt,
            serialize_message(step_msg),
            serialize_message(imu_msg),
            serialize_message(jointstate_msg),
            serialize_message(pressure_left),
            serialize_message(pressure_right))

        result = deserialize_message(stepi, JointCommand)
        return result

    def step_open_loop(self, dt: float, cmdvel_msg: Twist):
        if dt == 0.0:
            # preventing weird spline interpolation errors on edge case
            dt = 0.001
        stepi = self.py_walk_wrapper.step_open_loop(dt, serialize_message(cmdvel_msg))

        result = deserialize_message(stepi, PoseArray)
        return result

    def get_left_foot_pose(self):
        foot_pose = self.py_walk_wrapper.get_left_foot_pose()
        result = deserialize_message(foot_pose, Pose)
        return result

    def get_right_foot_pose(self):
        foot_pose = self.py_walk_wrapper.get_right_foot_pose()
        result = deserialize_message(foot_pose, Pose)
        return result

    def set_parameters(self, param_dict):
        parameters = parse_parameter_dict(namespace="", parameter_dict=param_dict)
        for parameter in parameters:
            self.py_walk_wrapper.set_parameter(serialize_message(parameter))

    def get_phase(self):
        return self.py_walk_wrapper.get_phase()

    def get_freq(self):
        return self.py_walk_wrapper.get_freq()

    def get_odom(self):
        odom = self.py_walk_wrapper.get_odom()
        result = deserialize_message(odom, Odometry)
        return result

    def publish_debug(self):
        self.py_walk_wrapper.publish_debug()
