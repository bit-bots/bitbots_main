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
        self.values = np.zeros((8, 10), dtype=np.float)
        self.current_index = 0

        rospy.Subscriber("/foot_pressure", FootPressure, self.pressure_cb)
        self.pressure_pub = rospy.Publisher("/foot_pressure_filtered", FootPressure, queue_size=1)
        rospy.Service("/set_foot_scale", FootScale, self.foot_scale_cb)
        rospy.Service("/set_foot_zero", Empty, self.zero_cb)
        rospy.Service("/reset_foot_calibration", Empty, self.reset_cb)
        rospy.spin()

    def pressure_cb(self, msg: FootPressure):
        if self.save_msgs:
            self.last_msgs.append(copy.copy(msg))

        self.values[0][self.current_index] = (msg.l_l_f - self.zero[0]) * self.scale[0]
        self.values[1][self.current_index] = (msg.l_l_b - self.zero[1]) * self.scale[1]
        self.values[2][self.current_index] = (msg.l_r_f - self.zero[2]) * self.scale[2]
        self.values[3][self.current_index] = (msg.l_r_b - self.zero[3]) * self.scale[3]
        self.values[4][self.current_index] = (msg.r_l_f - self.zero[4]) * self.scale[4]
        self.values[5][self.current_index] = (msg.r_l_b - self.zero[5]) * self.scale[5]
        self.values[6][self.current_index] = (msg.r_r_f - self.zero[6]) * self.scale[6]
        self.values[7][self.current_index] = (msg.r_r_b - self.zero[7]) * self.scale[7]

        msg.l_l_f = float(np.mean(self.values[0]))
        msg.l_l_b = float(np.mean(self.values[1]))
        msg.l_r_f = float(np.mean(self.values[2]))
        msg.l_r_b = float(np.mean(self.values[3]))
        msg.r_l_f = float(np.mean(self.values[4]))
        msg.r_l_b = float(np.mean(self.values[5]))
        msg.r_r_f = float(np.mean(self.values[6]))
        msg.r_r_b = float(np.mean(self.values[7]))

        self.current_index = (self.current_index + 1) % 10

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
        for i in range(8):
            self.zero[i] = (float(np.median(message_data[i])))

        rospy.set_param("~zero", self.zero)
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

        single_scale = request.weight / (np.mean(message_data[request.sensor]) - self.zero[request.sensor])
        rospy.logwarn("mean:" + str(np.median(message_data[request.sensor])))

        self.scale[request.sensor] = float(single_scale)
        rospy.set_param("~scale", self.scale)
        self.last_msgs = []
        self.save_yaml()
        return FootScaleResponse()

    def reset_cb(self,request):
        self.scale = [1.0] * 8
        self.zero = [0.0] * 8
        self.save_yaml()

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
