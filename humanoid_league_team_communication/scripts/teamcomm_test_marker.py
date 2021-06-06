#!/usr/bin/env python3

import traceback

import rospy
import copy
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from humanoid_league_msgs.msg import PoseWithCertaintyStamped, PoseWithCertaintyArray, \
    ObstacleRelative, ObstacleRelativeArray, PoseWithCertainty, TeamData
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from tf2_geometry_msgs import PointStamped
from tf.transformations import euler_from_quaternion
import tf2_ros
import numpy as np

ROBOT_HEIGHT = 0.8
ROBOT_DIAMETER = 0.2
ROBOT_SPEED = 0.3
BALL_DIAMETER = 0.13
OBSTACLE_NUMBER = 4
OBSTACLE_HEIGT = 0.8
OBSTACLE_DIAMETER = 0.2

rospy.init_node("team_comm_test_marker")
tf_buffer = tf2_ros.Buffer(rospy.Duration(30))
tf_listener = tf2_ros.TransformListener(tf_buffer)


class TeamCommMarker(object):
    def __init__(self, server):
        self.server = server
        self.pose = Pose()
        self.active = True
        self.confidence = 1.0
        self.int_marker = None
        self.make_marker()
        self.menu_handler = MenuHandler()
        item = self.menu_handler.insert("active", callback=self.menu_callback)
        self.menu_handler.setCheckState(item, MenuHandler.CHECKED)
        high_conf = self.menu_handler.insert("high confidence", callback=self.conf_callback)
        self.menu_handler.setCheckState(high_conf, MenuHandler.CHECKED)
        self.menu_handler.apply(self.server, self.marker_name)

    def feedback(self, feedback):
        self.pose = feedback.pose
        self.server.applyChanges()

    def menu_callback(self, feedback):
        item = feedback.menu_entry_id
        if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
            self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
            self.active = False
        else:
            self.active = True
            self.menu_handler.setCheckState(item, MenuHandler.CHECKED)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def id_callback(self, feedback):
        item = feedback.menu_entry_id
        if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
            # unchecking something should lead to id 1
            self.robot_id = 1
            self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
            self.menu_handler.setCheckState(2, MenuHandler.CHECKED)
        else:
            items = [2, 3, 4, 5]
            self.robot_id = item - 1
            items.remove(item)
            for item_id in items:
                self.menu_handler.setCheckState(item_id, MenuHandler.UNCHECKED)
            self.menu_handler.setCheckState(item, MenuHandler.CHECKED)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def conf_callback(self, feedback):
        item = feedback.menu_entry_id
        if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
            self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
            self.confidence = 0.3
        else:
            self.confidence = 1.0
            self.menu_handler.setCheckState(item, MenuHandler.CHECKED)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def make_marker(self):
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "map"
        self.int_marker.pose = self.pose
        self.int_marker.scale = 1

        self.int_marker.name = self.marker_name

        control = InteractiveMarkerControl()
        control.orientation.w = math.sqrt(2) / 2
        control.orientation.x = 0
        control.orientation.y = math.sqrt(2) / 2
        control.orientation.z = 0
        control.interaction_mode = self.interaction_mode
        self.int_marker.controls.append(copy.deepcopy(control))

        # make a box which also moves in the plane
        markers = self.make_individual_markers(self.int_marker)
        for marker in markers:
            control.markers.append(marker)
        control.always_visible = True
        self.int_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(self.int_marker, self.feedback)


class RobotMarker(TeamCommMarker):  # todo change color based on active
    def __init__(self, server):
        self.marker_name = "team_robot"
        self.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        super().__init__(server)
        self.pose.position.x = 3.0
        self.robot_id = 1
        id_1 = self.menu_handler.insert("id 1", callback=self.id_callback)
        id_2 = self.menu_handler.insert("id 2", callback=self.id_callback)
        id_3 = self.menu_handler.insert("id 3", callback=self.id_callback)
        id_4 = self.menu_handler.insert("id 4", callback=self.id_callback)
        self.menu_handler.setCheckState(id_1, MenuHandler.CHECKED)
        self.ball = BallMarker(server, self.robot_id)

    def make_individual_markers(self, msg):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale = Vector3(ROBOT_DIAMETER, ROBOT_DIAMETER, ROBOT_HEIGHT)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 0.4
        marker.pose.position = Point(0, 0, ROBOT_HEIGHT / 2)
        return (marker,)


class BallMarker(TeamCommMarker):
    def __init__(self, server, id):
        self.marker_name = f"team_ball_{id}"
        self.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        super().__init__(server)
        self.pose.position.x = 1.0

    def make_individual_markers(self, msg):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = BALL_DIAMETER
        marker.scale.y = BALL_DIAMETER
        marker.scale.z = BALL_DIAMETER
        marker.pose.position.z = BALL_DIAMETER / 2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 0.4
        return (marker,)


class ObstacleMarker(TeamCommMarker):
    def __init__(self, server, name):
        self.marker_name = name
        self.type = 0  # unknown
        self.player_number = 0
        self.confidence = 1.0
        self.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        super(ObstacleMarker, self).__init__(server)
        sub_menu_handle = self.menu_handler.insert("Color")
        h_mode_last = self.menu_handler.insert("red", parent=sub_menu_handle, callback=self.colorCb)
        h_mode_last = self.menu_handler.insert("blue", parent=sub_menu_handle, callback=self.colorCb)
        h_mode_last = self.menu_handler.insert("unknown", parent=sub_menu_handle, callback=self.colorCb)
        self.menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)
        self.menu_handler.apply(self.server, self.marker_name)
        self.pose.position.x = 2.0

    def colorCb(self, feedback):
        item = feedback.menu_entry_id
        if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
            # unchecking something should lead to unknown color
            self.type = 0
            self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
            self.menu_handler.setCheckState(5, MenuHandler.CHECKED)
            self.int_marker.controls[1].markers[0].type.r = 0.0
            self.int_marker.controls[1].markers[0].type.g = 0.0
            self.int_marker.controls[1].markers[0].type.b = 0.0
        else:
            if item == 3:
                self.type = 2
                self.menu_handler.setCheckState(4, MenuHandler.UNCHECKED)
                self.menu_handler.setCheckState(5, MenuHandler.UNCHECKED)
                self.int_marker.controls[1].markers[0].type.r = 1.0
                self.int_marker.controls[1].markers[0].type.g = 0.0
                self.int_marker.controls[1].markers[0].type.b = 0.0
            elif item == 4:
                self.type = 3
                self.menu_handler.setCheckState(3, MenuHandler.UNCHECKED)
                self.menu_handler.setCheckState(5, MenuHandler.UNCHECKED)
                self.int_marker.controls[1].markers[0].type.r = 0.0
                self.int_marker.controls[1].markers[0].type.g = 0.0
                self.int_marker.controls[1].markers[0].type.b = 1.0
            elif item == 5:
                self.type = 0
                self.menu_handler.setCheckState(3, MenuHandler.UNCHECKED)
                self.menu_handler.setCheckState(4, MenuHandler.UNCHECKED)
                self.int_marker.controls[1].markers[0].type.r = 0.0
                self.int_marker.controls[1].markers[0].type.g = 0.0
                self.int_marker.controls[1].markers[0].type.b = 0.0
            self.menu_handler.setCheckState(item, MenuHandler.CHECKED)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def make_individual_markers(self, msg):
        marker = Marker()

        marker.type = Marker.CYLINDER
        marker.scale = Vector3(POST_DIAMETER, POST_DIAMETER, OBSTACLE_HEIGT)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.4
        marker.pose.position = Point(0, 0, OBSTACLE_HEIGT / 2)

        return (marker,)

    def get_absolute_message(self):
        msg = ObstacleRelative()
        msg.pose.pose.pose.position = self.pose.position
        msg.height = OBSTACLE_HEIGT
        msg.width = OBSTACLE_DIAMETER
        msg.type = self.type
        msg.pose.confidence = self.confidence
        msg.playerNumber = self.player_number
        return msg


class ObstacleMarkerArray:
    def __init__(self, server, cam_info):
        self.cam_info = cam_info
        self.absolute_publisher = rospy.Publisher("obstacles_absolute", ObstacleRelativeArray, queue_size=1)
        self.relative_publisher = rospy.Publisher("obstacles_relative", ObstacleRelativeArray, queue_size=1)
        self.obstacles = []
        for i in range(0, OBSTACLE_NUMBER):
            self.obstacles.append(ObstacleMarker(server, cam_info, "obstacle_" + str(i)))


class TeamMessage:
    def __init__(self, robot):  # todo ball, obstacles
        self.robot = robot
        self.pub = rospy.Publisher("team_data", TeamData, queue_size=1)

    def publish(self, dt):
        if self.robot.active:
            msg = TeamData()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.robot_id = self.robot.robot_id
            msg.robot_position.pose.pose = self.robot.pose
            msg.robot_position.confidence = self.robot.confidence

            if self.robot.ball.active:
                try:
                    ball_point_stamped = PointStamped()
                    ball_point_stamped.header.stamp = rospy.Time.now()
                    ball_point_stamped.header.frame_id = "map"
                    ball_point_stamped.point = self.ball.pose.position
                    ball_in_camera_optical_frame = tf_buffer.transform(ball_point_stamped, "map",
                                                                       timeout=rospy.Duration(0.5))
                    ball_in_footprint_frame = tf_buffer.transform(ball_in_camera_optical_frame, "base_footprint",
                                                                  timeout=rospy.Duration(0.5))
                    ball_relative = PoseWithCertainty()
                    ball_relative.pose.pose.position = ball_in_footprint_frame.point
                    ball_relative.pose.pose.orientation = Quaternion(0, 0, 0, 1)
                    ball_relative.confidence = self.ball.confidence
                    msg.ball_relative = ball_relative

                    cartesian_distance = math.sqrt(
                        ball_relative.pose.pose.position.x ** 2 + ball_relative.pose.pose.position.y ** 2)
                    msg.time_to_position_at_ball = cartesian_distance / ROBOT_SPEED
                except tf2_ros.LookupException as ex:
                    rospy.logwarn_throttle(10.0, rospy.get_name() + ": " + str(ex))
                    return None
                except tf2_ros.ExtrapolationException as ex:
                    rospy.logwarn_throttle(10.0, rospy.get_name() + ": " + str(ex))
                    return None
                self.pub.publish(msg)
            else:
                # ball not seen
                msg.ball_relative.confidence = 0.0
                msg.time_to_position_at_ball = 0.0

            self.pub.publish(msg)
        # todo obstacles


if __name__ == "__main__":
    # retrieve InteractiveMarkerServer and setup subscribers and publishers
    server = InteractiveMarkerServer("basic_controls")
    robot = RobotMarker(server)
    server.applyChanges()

    team_message = TeamMessage(robot)
    # create a timer to update the published ball transform
    rospy.Timer(rospy.Duration(0.1), team_message.publish)
    # run and block until finished
    rospy.spin()
