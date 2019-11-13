#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""
import traceback

import rospy
import copy
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from humanoid_league_msgs.msg import BallRelative, GoalRelative
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import tf2_ros
import numpy as np

rospy.init_node("imu_interactive_marker")

SIZE = 0.3


class IMUMarker:

    def __init__(self, server):
        self.marker_name = "IMU"
        self.imu_publisher = rospy.Publisher("/imu/data", Imu, queue_size=1)
        self.server = server
        self.pose = Pose()
        self.pose.orientation.w = 1

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "imu_frame"
        int_marker.pose = self.pose
        int_marker.scale = 1
        int_marker.name = self.marker_name
        int_marker.description = "Rotate 3DOF to simulate IMU orientation"

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.always_visible = True
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(int_marker, self.processFeedback)

    def processFeedback(self, feedback):
        self.pose = feedback.pose

    def publish_marker(self, e):
        # create IMU message and publis
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.get_rostime()
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation = self.pose.orientation

        self.imu_publisher.publish(imu_msg)



if __name__ == "__main__":
    server = InteractiveMarkerServer("interactive_imu")
    imu = IMUMarker(server)

    server.applyChanges()

    # create a timer to update the published ball transform
    rospy.Timer(rospy.Duration(0.1), imu.publish_marker)

    # run and block until finished
    rospy.spin()
