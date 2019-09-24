#!/usr/bin/env python2

import rospy
from bitbots_msgs.msg import FootPressure
from bitbots_msgs.srv import FootScale, FootScaleRequest, FootScaleResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import PointStamped, TransformStamped
from geometry_msgs.msg import WrenchStamped
import copy
import numpy as np
import yaml
import rospkg
import tf2_ros
import os
from dynamic_reconfigure.server import Server
from bitbots_ros_control.cfg import bitbots_ros_control_paramsConfig


class PressureConverter:
    def __init__(self):
        rospy.init_node("pressure_converter")

        self.zero = rospy.get_param("~zero")
        self.scale = rospy.get_param("~scale")

        self.last_msgs = [[], []] # last messages for estimating the zero position and scaling factor
        self.save_msgs = False  # whether to save messages or not
        self.values = np.zeros((8, 10), dtype=np.float)
        self.current_index_right = 0
        self.current_index_left = 0
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pressure_pub_left  = rospy.Publisher("/foot_pressure_left/filtered", FootPressure, queue_size=1)
        self.pressure_pub_right = rospy.Publisher("/foot_pressure_right/filtered", FootPressure, queue_size=1)

        self.cop_l_pub = rospy.Publisher("/cop_l", PointStamped, queue_size=1)
        self.cop_r_pub = rospy.Publisher("/cop_r", PointStamped, queue_size=1)

        self.wrench_pubs = []
        self.wrenches = []
        cleats_and_cop = ["l_cleat_l_front",
                          "l_cleat_l_back",
                          "l_cleat_r_front",
                          "l_cleat_r_back",
                          "r_cleat_l_front",
                          "r_cleat_l_back",
                          "r_cleat_r_front",
                          "r_cleat_r_back",
                          "cop_l",
                          "cop_r"]
        for i in range(10):
            wrench = WrenchStamped()
            wrench.header.frame_id = cleats_and_cop[i]
            self.wrenches.append(wrench)
            self.wrench_pubs.append(rospy.Publisher("/wrench" + cleats_and_cop[i], WrenchStamped, queue_size=1))

        rospy.Service("/set_foot_scale", FootScale, self.foot_scale_cb)
        rospy.Service("/set_foot_zero", Empty, self.zero_cb)
        rospy.Service("/reset_foot_calibration", Empty, self.reset_cb)

        self.threshold = 0.0
        self.dyn_reconf = Server(bitbots_ros_control_paramsConfig, self.reconfigure)

        rospy.Subscriber("/foot_pressure_left/raw", FootPressure, self.pressure_cb_left)
        rospy.Subscriber("/foot_pressure_right/raw", FootPressure, self.pressure_cb_right)

        rospy.spin()

    def reconfigure(self, config, level):
        """ Dynamic reconfigure """
        # just pass on to the StandupHandler, as all the variables are located there
        self.threshold = config.cop_threshold
        return config

    def pressure_cb_left(self, msg):
        if self.save_msgs:
            self.last_msgs.append(copy.copy(msg))

        self.values[0][self.current_index_left] = (msg.left_front  - self.zero[0]) * self.scale[0]
        self.values[1][self.current_index_left] = (msg.left_back   - self.zero[1]) * self.scale[1]
        self.values[2][self.current_index_left] = (msg.right_front - self.zero[2]) * self.scale[2]
        self.values[3][self.current_index_left] = (msg.right_back  - self.zero[3]) * self.scale[3]

        for i in range(4):
            # past dated for tf
            self.wrenches[i].header.stamp = msg.header.stamp - rospy.Duration(0.1)
            self.wrenches[i].wrench.force.z = max(float(np.mean(self.values[i])), 0)
            self.wrench_pubs[i].publish(self.wrenches[i])

        msg.left_front  = self.wrenches[0].wrench.force.z
        msg.left_back   = self.wrenches[1].wrench.force.z
        msg.right_front = self.wrenches[2].wrench.force.z
        msg.right_back  = self.wrenches[3].wrench.force.z

        self.current_index_left = (self.current_index_left + 1) % 10

        self.pressure_pub_left.publish(msg)

        pos = [0.085, 0.045]
        cop_l = PointStamped()
        cop_l.header.frame_id = "l_sole"
        cop_l.header.stamp = msg.header.stamp
        sum_of_forces_l = msg.left_front + msg.left_back + msg.right_front + msg.right_back
        if sum_of_forces_l > self.threshold:
            cop_l.point.x = (msg.left_front * pos[0] - msg.left_back * pos[0] + msg.right_front * pos[0] - msg.right_back * pos[0]) / sum_of_forces_l
            cop_l.point.y = (msg.left_front * pos[1] + msg.left_back * pos[1] - msg.right_front * pos[1] - msg.right_back * pos[1]) / sum_of_forces_l
        else:
            cop_l.point.x = 0
            cop_l.point.y = 0
        self.cop_l_pub.publish(cop_l)

        # past dated for tf
        self.wrenches[8].header.stamp = msg.header.stamp - rospy.Duration(0.05)
        self.wrenches[8].wrench.force.z = sum_of_forces_l

        cop_l_tf = TransformStamped()
        cop_l_tf.header = cop_l.header
        cop_l_tf.child_frame_id = "cop_l"
        cop_l_tf.transform.translation.x = cop_l.point.x
        cop_l_tf.transform.translation.y = cop_l.point.y
        cop_l_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(cop_l_tf)

        # do this last so rviz can get the tf first
        self.wrench_pubs[8].publish(self.wrenches[8])

    def pressure_cb_right(self, msg):
        self.values[4][self.current_index_right] = (msg.left_front  - self.zero[4]) * self.scale[4]
        self.values[5][self.current_index_right] = (msg.left_back   - self.zero[5]) * self.scale[5]
        self.values[6][self.current_index_right] = (msg.right_front - self.zero[6]) * self.scale[6]
        self.values[7][self.current_index_right] = (msg.right_back  - self.zero[7]) * self.scale[7]


        for i in range(4):
            # past dated for tf
            self.wrenches[i+4].header.stamp = msg.header.stamp - rospy.Duration(0.1)
            self.wrenches[i+4].wrench.force.z = max(float(np.mean(self.values[i+4])), 0)
            self.wrench_pubs[i+4].publish(self.wrenches[i+4])

        msg.left_front  = self.wrenches[4].wrench.force.z
        msg.left_back   = self.wrenches[5].wrench.force.z
        msg.right_front = self.wrenches[6].wrench.force.z
        msg.right_back  = self.wrenches[7].wrench.force.z

        self.current_index_right = (self.current_index_right + 1) % 10

        self.pressure_pub_right.publish(msg)

        pos = [0.085, 0.045]
        cop_r = PointStamped()
        cop_r.header.frame_id = "r_sole"
        cop_r.header.stamp = msg.header.stamp
        sum_of_forces_r = msg.left_front + msg.left_back + msg.right_back + msg.right_front
        if sum_of_forces_r > self.threshold:
            cop_r.point.x = (msg.left_front * pos[0] - msg.left_back * pos[0] + msg.right_front * pos[0] - msg.right_back * pos[0]) / sum_of_forces_r
            cop_r.point.y = (msg.left_front * pos[1] + msg.left_back * pos[1] - msg.right_front * pos[1] - msg.right_back * pos[1]) / sum_of_forces_r
        else:
            cop_r.point.x = 0
            cop_r.point.y = 0
        self.cop_r_pub.publish(cop_r)

        # past dated for tf
        self.wrenches[9].header.stamp = msg.header.stamp - rospy.Duration(0.05)
        self.wrenches[9].wrench.force.z = sum_of_forces_r

        cop_r_tf = TransformStamped()
        cop_r_tf.header = cop_r.header
        cop_r_tf.child_frame_id = "cop_r"
        cop_r_tf.transform.translation.x = cop_r.point.x
        cop_r_tf.transform.translation.y = cop_r.point.y
        cop_r_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(cop_r_tf)

        # do this last so rviz can get the tf first
        self.wrench_pubs[9].publish(self.wrenches[9])

    def zero_cb(self, request):
        # save messages
        self.save_msgs = True
        while min(len(self.last_msgs[0]), len(self.last_msgs[1])) < 100:
            rospy.sleep(0.1)
            rospy.logerr_throttle(0.5, "Collecting messages for Zeroing")
        self.save_msgs = False
        rospy.sleep(0.1) # process last requests for saving messages

        message_data = np.zeros((8, min(len(self.last_msgs[0]), len(self.last_msgs[1]))))

        for i in range(min(len(self.last_msgs[0]), len(self.last_msgs[1]))):
            message_data[0][i] = self.last_msgs[0][i].left_front
            message_data[1][i] = self.last_msgs[0][i].left_back
            message_data[2][i] = self.last_msgs[0][i].right_front
            message_data[3][i] = self.last_msgs[0][i].right_back
            message_data[4][i] = self.last_msgs[1][i].left_front
            message_data[5][i] = self.last_msgs[1][i].left_back
            message_data[6][i] = self.last_msgs[1][i].right_front
            message_data[7][i] = self.last_msgs[1][i].right_back

        self.last_msgs = [[], []]
        for i in range(8):
            self.zero[i] = (float(np.median(message_data[i])))

        rospy.set_param("~zero", self.zero)
        self.save_yaml()
        return EmptyResponse()

    def foot_scale_cb(self, request):
        # save messages
        self.save_msgs = True
        while min(len(self.last_msgs[0]), len(self.last_msgs[1])) < 100:
            rospy.sleep(0.1)
            rospy.logerr_throttle(0.5, "Collecting messages for Scale correction")
        self.save_msgs = False
        rospy.sleep(0.1) # process last requests for saving messages

        # this is a bit inefficient but it works
        message_data = np.zeros((8, min(len(self.last_msgs[0]), len(self.last_msgs[1]))))

        for i in range(min(len(self.last_msgs[0]), len(self.last_msgs[1]))):
            message_data[0][i] = self.last_msgs[0][i].left_front
            message_data[1][i] = self.last_msgs[0][i].left_back
            message_data[2][i] = self.last_msgs[0][i].right_front
            message_data[3][i] = self.last_msgs[0][i].right_back
            message_data[4][i] = self.last_msgs[1][i].left_front
            message_data[5][i] = self.last_msgs[1][i].left_back
            message_data[6][i] = self.last_msgs[1][i].right_front
            message_data[7][i] = self.last_msgs[1][i].right_back

        single_scale = request.weight / (np.mean(message_data[request.sensor]) - self.zero[request.sensor])
        rospy.logwarn("mean:" + str(np.median(message_data[request.sensor])))

        self.scale[request.sensor] = float(single_scale)
        rospy.set_param("~scale", self.scale)
        self.last_msgs = [[], []]
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
