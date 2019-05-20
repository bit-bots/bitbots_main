#!/usr/bin/env python3

import rospy
from bitbots_msgs.msg import FootPressure
from bitbots_msgs.srv import FootScale, FootScaleRequest, FootScaleResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import copy
import numpy as np
import yaml
import rospkg
import os

class PressureConverter:
    def __init__(self):
        rospy.init_node("pressure_converter")

        self.zero = rospy.get_param("~zero")
        self.scale = rospy.get_param("~scale")

        self.last_msgs = [] #  last messages for estimating the zero position and scaling factor
        self.save_msgs = False # whether to save messages or not

        rospy.Subscriber("/foot_pressure", FootPressure, self.pressure_cb)
        self.pressure_pub = rospy.Publisher("/foot_pressure_filtered", FootPressure, queue_size=1)
        rospy.Service("/set_foot_scale", FootScale, self.foot_scale_cb)
        rospy.Service("/set_foot_zero", Empty, self.zero_cb)
        rospy.spin()

    def pressure_cb(self, msg: FootPressure):
        if self.save_msgs:
            self.last_msgs.append(copy.copy(msg))

        msg.l_l_f = (msg.l_l_f - self.zero[0]) * self.scale[0]
        msg.l_l_b = (msg.l_l_b - self.zero[1]) * self.scale[1]
        msg.l_r_f = (msg.l_r_f - self.zero[2]) * self.scale[2]
        msg.l_r_b = (msg.l_r_b - self.zero[3]) * self.scale[3]
        msg.r_l_f = (msg.r_l_f - self.zero[4]) * self.scale[4]
        msg.r_l_b = (msg.r_l_b - self.zero[5]) * self.scale[5]
        msg.r_r_f = (msg.r_r_f - self.zero[6]) * self.scale[6]
        msg.r_r_b = (msg.r_r_b - self.zero[7]) * self.scale[7]

        self.pressure_pub.publish(msg)

    def zero_cb(self, request: EmptyRequest):
        # save messages
        self.save_msgs = True
        while len(self.last_msgs) < 100:
            rospy.sleep(0.1)
            rospy.logerr_throttle(0.5, "Collecting messages for Zeroing")
        self.save_msgs = False
        rospy.sleep(0.1) # process last requests for saving messages

        message_data = np.zeros((8, len(self.last_msgs)))

        for i in range(len(self.last_msgs)):
            message_data[0][i] = self.last_msgs[i].l_l_f
            message_data[1][i] = self.last_msgs[i].l_l_b
            message_data[2][i] = self.last_msgs[i].l_r_f
            message_data[3][i] = self.last_msgs[i].l_r_b
            message_data[4][i] = self.last_msgs[i].r_l_f
            message_data[5][i] = self.last_msgs[i].r_l_b
            message_data[6][i] = self.last_msgs[i].r_r_f
            message_data[7][i] = self.last_msgs[i].r_r_b

        self.last_msgs = []
        zero = []
        for i in range(8):
            zero.append(float(np.median(message_data[i])))

        rospy.set_param("~zero", zero)
        self.save_yaml()
        return EmptyResponse()

    def foot_scale_cb(self, request: FootScaleRequest):
        # save messages
        self.save_msgs = True
        while len(self.last_msgs) < 100:
            rospy.sleep(0.1)
            rospy.logerr_throttle(0.5, "Collecting messages for Scale correction")
        self.save_msgs = False
        rospy.sleep(0.1) # process last requests for saving messages

        # this is a bit inefficient but it works
        message_data = np.zeros((8, len(self.last_msgs)))

        for i in range(len(self.last_msgs)):
            message_data[0][i] = self.last_msgs[i].l_l_f
            message_data[1][i] = self.last_msgs[i].l_l_b
            message_data[2][i] = self.last_msgs[i].l_r_f
            message_data[3][i] = self.last_msgs[i].l_r_b
            message_data[4][i] = self.last_msgs[i].r_l_f
            message_data[5][i] = self.last_msgs[i].r_l_b
            message_data[6][i] = self.last_msgs[i].r_r_f
            message_data[7][i] = self.last_msgs[i].r_r_b

        single_scale = np.median(message_data[request.sensor]) / request.weight
        self.scale[request.sensor] = float(single_scale)
        rospy.set_param("~scale", self.scale)
        self.last_msgs = []
        self.save_yaml()
        return FootScaleResponse()

    def save_yaml(self):
        data = {'scale': self.scale,
                'zero': self.zero}

        r = rospkg.RosPack()
        path = r.get_path("bitbots_ros_control")
        with open(path + "/config/pressure_" + "amy" + ".yaml", 'w') as f:
            rospy.loginfo("pressure sensor configuration saved")
            yaml.dump(data, f)
            f.close()


if __name__ == "__main__":
    p = PressureConverter()
