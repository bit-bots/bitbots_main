import pickle
import time

from sklearn.preprocessing import StandardScaler
import tf
from sensor_msgs.msg import JointState, Imu, Image
from geometry_msgs.msg import Point
import math
import numpy as np

class FallClassifier:

    def __init__(self, path, smooth_threshold=10):
        with open(path + "classifier.pkl", 'rb') as file:
            self.classifier = pickle.load(file)
        with open(path + "scaler.pkl", 'rb') as file:
            self.scaler: StandardScaler = pickle.load(file)
        with open(path + "types.pkl", 'rb') as file:
            self.types = pickle.load(file)

        print(F'{self.classifier}  {self.types}')

        self.counter = 0
        self.last_prediction = 0
        self.smooth_threshold = smooth_threshold

    def classify(self, imu, joint_state, cop_l, cop_r):
        start_time = time.time()
        data = get_data_from_msgs(imu, joint_state, cop_l, cop_r,
                                  imu_raw=self.types['imu_raw'],
                                  imu_orient=self.types['imu_orient'], joint_states=self.types['joint_states'],
                                  imu_fused=self.types['imu_fused'], cop=self.types['cop'])
        scaled_date = self.scaler.transform([data])
        result = self.classifier.predict(scaled_date)
        #print((time.time() - start_time) * 1000)
        return result[0]

    def smooth_classify(self, imu, joint_state, cop_l, cop_r):
        # only predict a fall if we classified this 10 times straight
        predition = self.classify(imu, joint_state, cop_l, cop_r)
        if predition == self.last_prediction and predition != 0:
            self.counter += 1
            if self.counter > self.smooth_threshold:
                return predition
            else:
                return 0
        else:
            self.counter = 0
            self.last_prediction = predition

def get_data_from_msgs(imu_msg, joint_state_msg, cop_l_msg, cop_r_msg, imu_raw=True, imu_orient=True, joint_states=True,
                       imu_fused=True, cop=True):
    data = []
    if imu_raw:
        data.append(imu_msg.linear_acceleration.x)
        data.append(imu_msg.linear_acceleration.y)
        data.append(imu_msg.linear_acceleration.z)
        data.append(imu_msg.angular_velocity.x)
        data.append(imu_msg.angular_velocity.y)
        data.append(imu_msg.angular_velocity.z)
    if imu_orient:
        euler = tf.transformations.euler_from_quaternion(
            [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
        data.append(euler[0])
        data.append(euler[1])
        data.append(euler[2])

    if joint_states:
        for i in range(len(joint_state_msg.name)):
            # only add leg joints
            if joint_state_msg.name[i] in ["RHipPitch", "RHipRoll", "RKnee", "RAnklePitch", "RAnkleRoll",
                                           "LHipPitch", "LHipRoll", "LKnee", "LAnklePitch", "LAnkleRoll"]:
                data.append(joint_state_msg.effort[i])
    if imu_fused:
        fused_rpy = fused_from_quat(imu_msg.orientation)
        data.append(fused_rpy[0])
        data.append(fused_rpy[1])

    if cop:
        data.append(cop_l_msg.point.x)
        data.append(cop_l_msg.point.y)
        data.append(cop_r_msg.point.x)
        data.append(cop_r_msg.point.y)
    return data

# Python version of this code https://github.com/AIS-Bonn/rot_conv_lib/blob/master/src/rot_conv.cpp
def fused_from_quat(q):
    # Fused yaw of Quaternion
    fused_yaw = 2.0 * math.atan2(q.z, q.w)  # Output of atan2 is [-pi,pi], so this expression is in [-2*pi,2*pi]
    if fused_yaw > math.pi:
        fused_yaw -= 2 * math.pi  # fused_yaw is now in[-2 * pi, pi]
    if fused_yaw <= -math.pi:
        fused_yaw += 2 * math.pi  # fused_yaw is now in (-pi, pi]

    # Calculate the fused pitch and roll
    stheta = 2.0 * (q.y * q.w - q.x * q.z)
    sphi = 2.0 * (q.y * q.z + q.x * q.w)
    if stheta >= 1.0:  # Coerce stheta to[-1, 1]
        stheta = 1.0
    elif stheta <= -1.0:
        stheta = -1.0
    if sphi >= 1.0:  # Coerce sphi to[-1, 1]
        sphi = 1.0
    elif sphi <= -1.0:
        sphi = -1.0
    fused_pitch = math.asin(stheta)
    fused_roll = math.asin(sphi)

    return fused_roll, fused_pitch, fused_yaw
