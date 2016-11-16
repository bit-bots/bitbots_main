import numpy as np
from cython.operator cimport dereference as deref
from bitbots_common.util import get_config

cdef double pi = 3.1415926535897932384626433832795028841971693
cdef double rad_to_degree = 180.0 / pi
cdef double degree_to_rad = pi / 180.0

cdef dict config = get_config()
cdef list foot_size_config = config[config["RobotTypeName"]]["foot_size"]
cdef Matrix2d foot_size
foot_size.insert(<double>foot_size_config[0][0]).add(<double>foot_size_config[1][0]).add(<double>foot_size_config[0][1]).add(<double>foot_size_config[1][1])

#rad_to_degree
def rtd(rad=1):
    return rad * rad_to_degree
#degree_to_rad
def dtr(deg=1):
    return deg * degree_to_rad

cpdef np.ndarray get_robot_horizon_p(Robot robot, int phase):
    cdef unsigned char c_phase = phase
    cdef Vector2d horizon_result = get_robot_horizon(deref(robot.robot), phase)
    return np.array((horizon_result.x(), horizon_result.y()))

cpdef np.ndarray get_robot_horizon_p_deg(Robot robot, int phase):
    cdef np.ndarray horizon_result = get_robot_horizon_p(robot, phase)
    return np.array((horizon_result[0] * rad_to_degree, horizon_result[1] * rad_to_degree))

cpdef np.ndarray get_camera_position_p (Robot robot, int phase):
    cdef unsigned char c_phase = phase
    cdef Vector3d camera_result = get_camera_position(deref(robot.robot), phase)
    return np.array((camera_result.x(), camera_result.y(), camera_result.z()))

# Wurde für den Sensoranteil im kinematischen Horizont verwendet, ist im Verhalten aber viel zu instabil @Robert IranOpen 2016
cpdef np.ndarray get_robot_horizon_r(Robot robot, PyDataVector rnpy):
    cdef Vector2d horizon_result = get_robot_horizon(deref(robot.robot), Vector3d(rnpy.get_x(), -rnpy.get_y(), rnpy.get_z()))
    return np.array((horizon_result.x(), horizon_result.y()))

cpdef get_head_angles_for_distance(Robot robot, np.ndarray pos, int left_leg):
    cdef Vector2d pos_v = Vector2d(<double>pos[0], <double>pos[1]) / 1000.0
    cdef bool left = left_leg != 0
    get_head_angles_for_distance_(deref(robot.robot), pos_v, left)

cpdef get_head_angles_for_distance_with_y_offset(Robot robot, np.ndarray pos, int left_leg, float y_angle):
    cdef Vector2d pos_v = Vector2d(<double>pos[0], <double>pos[1]) / 1000.0
    cdef bool left = left_leg != 0
    get_head_angles_for_distance_(deref(robot.robot), pos_v, left, y_angle)

cpdef get_head_angles_for_distance_with_y_x_offset(Robot robot, np.ndarray pos, int left_leg, float y_angle, float x_angle):
    cdef Vector2d pos_v = Vector2d(<double>pos[0], <double>pos[1]) / 1000.0
    cdef bool left = left_leg != 0
    get_head_angles_for_distance_(deref(robot.robot), pos_v, left, y_angle, x_angle)

cdef tuple kinematic_robot_angle(Robot robot, int phase):
    cdef Vector2d angle = calculate_kinematic_robot_angle(deref(robot.robot), phase) * rad_to_degree
    return (angle.x(), angle.y())
