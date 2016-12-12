from trajectory_msgs.msg import JointTrajectory

from bitbots_common.eigen cimport *

from bitbots_common.utilCython.datavector cimport IntDataVector as CIntDataVector
from bitbots_common.utilCython.datavector cimport DataVector as CDataVector
from bitbots_common.utilCython.pydatavector cimport PyIntDataVector as IntDataVector
from bitbots_common.utilCython.pydatavector cimport PyDataVector as DataVector
from bitbots_common.robot.kinematics cimport Robot
from libcpp cimport bool
from bitbots_common.pose.pypose cimport PyPose as Pose


from .cm730 cimport CM730

cdef class cm730_node(object):

    cdef object joint_publisher
    cdef object speak_publisher
    cdef object tem_publisher
    cdef object imu_publisher
    cdef object motor_power_service

    cdef list motor_goals
    cdef CM730 cm_730_object

    cpdef update_forever(self)
    cpdef update_once(self)
    cpdef update_motor_goals(self, object msg)
    cpdef update_sensor_data(self)
    cpdef switch_motor_power_service_call(self, object req)
    cpdef publish_joints(self, Pose robo_pose)
    cpdef publish_additional_servo_data(self, list temps, list voltages)
    cpdef publish_IMU(self, double gyro, double accel)
    cpdef publish_buttons(self, bool button1, bool button2)

cdef extern from "cmath" namespace "std":
    double asin(double)
    double acos(double)

cdef double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164;
cdef double degree_to_rad = pi / 180;
cdef double rad_to_degree = 180 / pi;

cdef inline double calc_sin_angle(const Vector3d& fst, const Vector3d& sec):
    if(fst.norm() == 0 or sec.norm() == 0):
        return 0 #TODO Rückgabewert sinvoll?
    return asin(fst.dot(sec) / (fst.norm() * sec.norm())) * rad_to_degree

cdef inline double calc_cos_angle(const Vector3d& fst, const Vector3d& sec):
    return acos(fst.dot(sec) / (fst.norm() * sec.norm())) * rad_to_degree

cdef inline CDataVector calculate_robot_angles(const CIntDataVector& rawData):
    cdef Vector3d raw = Vector3d(rawData.get_x(), rawData.get_y(), rawData.get_z())
    cdef double roll_angle, pitch_angle, yaw_angle

    pitch_angle = calc_sin_angle(raw, unitY3d())
    if(raw.z() < 0 and raw.y() < 0):
        pitch_angle = - pitch_angle - 180
    elif(raw.z() < 0 and raw.y() > 0):
        pitch_angle = 180 - pitch_angle

    roll_angle = calc_sin_angle(raw, unitX3d())

    #TODO mir ist noch keiner schlaue Formel für diesen Wingkel eingefallen
    yaw_angle = 0

    #print "pitch %f, roll %f" % (pitch_angle, roll_angle)

    return CDataVector(-roll_angle, -pitch_angle, yaw_angle)
