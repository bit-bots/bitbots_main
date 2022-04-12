#!/usr/bin/env python3
import tf
from sensor_msgs.msg import JointState, Imu, Image
from geometry_msgs.msg import Point

import math
import numpy as np
from bitbots_hcm.fall_classifier import get_data_from_msgs


class Dataset:

    def __init__(self):
        self.frames = []
        self.fall_impact_time = None
        self.fall_type = None

    def append_frame(self, time, imu, joint_states, cop_l, cop_r, image):
        frame = Frame(time, imu, joint_states, cop_l, cop_r, image)
        self.frames.append(frame)

    def as_scikit_data(self, imu_raw=True, imu_orient=True, joint_states=True, imu_fused=True, cop=True):
        X = []
        y = []
        for frame in self.frames:
            X.append(
                frame.get_data(imu_raw=imu_raw, imu_orient=imu_orient, joint_states=joint_states, imu_fused=imu_fused,
                               cop=cop))
            y.append(frame.label)
        return X, y

    def get_frame_index_by_time(self, time):
        difference = np.inf
        index = None
        for i in range(0, len(self.frames)):
            if abs(time - self.frames[i].time) < difference:
                index = i
                difference = abs(time - self.frames[i].time)
        return index

    def set_all_labels_to(self, label):
        if label == "stable":
            int_label = 0
        elif label == "front":
            int_label = 1
        elif label == "back":
            int_label = 2
        elif label == "left":
            int_label = 3
        elif label == "right":
            int_label = 4
        else:
            print("Wrong label")
            exit(1)

        for frame in self.frames:
            frame.label = int_label

    def append_dataset(self, dataset):
        self.frames = self.frames + dataset.frames


class Frame:

    def __init__(self, time, joint_states, imu, cop_l, cop_r, image):
        self.time = time
        self.imu: Imu = imu
        self.joint_states: JointState = joint_states
        self.cop_l: Point = cop_l
        self.cop_r: Point = cop_r
        self.image: Image = image
        self.label = -1

    def get_label(self):
        if self.label == -2:
            return "removed"
        elif self.label == -1:
            return "unlabeled"
        elif self.label == 0:
            return "stable"
        elif self.label == 1:
            return "front"
        elif self.label == 2:
            return "back"
        elif self.label == 3:
            return "left"
        elif self.label == 4:
            return "right"
        else:
            print(F"Label number {self.label} not clear, will use unlabeled")
            return "unlabeled"

    def get_data(self, imu_raw=True, imu_orient=True, joint_states=True, imu_fused=True, cop=True):
        return get_data_from_msgs(self.imu, self.joint_states, self.cop_l, self.cop_r, imu_raw, imu_orient,
                                  joint_states, imu_fused, cop)
