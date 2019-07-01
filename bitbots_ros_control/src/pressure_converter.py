#!/usr/bin/env python2

import rospy
from bitbots_msgs.msg import FootPressure
from bitbots_msgs.srv import FootScale, FootScaleRequest, FootScaleResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import PointStamped, TransformStamped
import copy
import numpy as np
import yaml
import rospkg
import tf2_ros
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
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/foot_pressure", FootPressure, self.pressure_cb)
        self.pressure_pub = rospy.Publisher("/foot_pressure_filtered", FootPressure, queue_size=1)

        self.cop_l_pub = rospy.Publisher("/cop_l", PointStamped, queue_size=1)
        self.cop_r_pub = rospy.Publisher("/cop_r", PointStamped, queue_size=1)
        rospy.Service("/set_foot_scale", FootScale, self.foot_scale_cb)
        rospy.Service("/set_foot_zero", Empty, self.zero_cb)
        rospy.Service("/reset_foot_calibration", Empty, self.reset_cb)

        self.threshold = 0
        self.dyn_reconf = Server(bitbots_ros_control_paramsConfig, self.reconfigure)

        rospy.spin()

    def reconfigure(self, config, level):
        """ Dynamic reconfigure """
        # just pass on to the StandupHandler, as all the variables are located there
        self.threshold = config.cop_threshold
        return config

    def pressure_cb(self, msg):
        # type:(FootPressure) -> None
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

        msg.l_l_f = max(float(np.mean(self.values[0])), 0)
        msg.l_l_b = max(float(np.mean(self.values[1])), 0)
        msg.l_r_f = max(float(np.mean(self.values[2])), 0)
        msg.l_r_b = max(float(np.mean(self.values[3])), 0)
        msg.r_l_f = max(float(np.mean(self.values[4])), 0)
        msg.r_l_b = max(float(np.mean(self.values[5])), 0)
        msg.r_r_f = max(float(np.mean(self.values[6])), 0)
        msg.r_r_b = max(float(np.mean(self.values[7])), 0)

        self.current_index = (self.current_index + 1) % 10

        self.pressure_pub.publish(msg)

        pos = [0.085, 0.045]
        cop_l = PointStamped()
        cop_l.header.frame_id = "l_sole"
        cop_l.header.stamp = msg.header.stamp
        sum_of_forces = msg.l_l_f + msg.l_l_b +  msg.l_r_f + msg.l_r_b
        if sum_of_forces > self.threshold:
            cop_l.point.x = (msg.l_l_f * pos[0] - msg.l_l_b * pos[0] + msg.l_r_f * pos[0] - msg.l_r_b * pos[0]) / sum_of_forces
            cop_l.point.y = (msg.l_l_f * pos[1] + msg.l_l_b * pos[1] - msg.l_r_f * pos[1] - msg.l_r_b * pos[1]) / sum_of_forces
        else:
            cop_l.point.x = 0
            cop_l.point.y = 0
        self.cop_l_pub.publish(cop_l)


        cop_r = PointStamped()
        cop_r.header.frame_id = "r_sole"
        cop_r.header.stamp = msg.header.stamp
        sum_of_forces = msg.r_l_f + msg.r_l_b + msg.r_r_b + msg.r_r_f
        if sum_of_forces > self.threshold:
            cop_r.point.x = (msg.r_l_f * pos[0] - msg.r_l_b * pos[0] + msg.r_r_f * pos[0] - msg.r_r_b * pos[0]) / sum_of_forces
            cop_r.point.y = (msg.r_l_f * pos[1] + msg.r_l_b * pos[1] - msg.r_r_f * pos[1] - msg.r_r_b * pos[1]) / sum_of_forces
        else:
            cop_r.point.x = 0
            cop_r.point.y = 0
        self.cop_r_pub.publish(cop_r)

    cop_r_tf = TransformStamped()
        cop_r_tf.header = cop_r.header
        cop_r_tf.child_frame_id = "cop_r"
        cop_r_tf.transform.translation.x = cop_r.point.x
        cop_r_tf.transform.translation.y = cop_r.point.y
        cop_r_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(cop_r_tf)

        cop_l_tf = TransformStamped()
        cop_l_tf.header = cop_l.header
        cop_l_tf.child_frame_id = "cop_l"
        cop_l_tf.transform.translation.x = cop_l.point.x
        cop_l_tf.transform.translation.y = cop_l.point.y
        cop_l_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(cop_l_tf)

    def zero_cb(self, request):
        #EmptyRequest):
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

    def foot_scale_cb(self, request):
        #FootScaleRequest):
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
        with open(path + "/config/pressure_" + os.getenv("ROBOT_NAME") + ".yaml", 'w') as f:
            rospy.loginfo("pressure sensor configuration saved")
            yaml.dump(data, f)
            f.close()


if __name__ == "__main__":
    p = PressureConverter()
