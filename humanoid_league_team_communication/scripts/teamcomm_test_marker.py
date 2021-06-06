#!/usr/bin/env python3

import traceback

import rospy
import copy
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from transforms3d.affines import compose, decompose
from transforms3d.quaternions import quat2mat, mat2quat
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
        self.menu_handler.apply(self.server, self.marker_name)
        self.ball = BallMarker(server, self.robot_id)

    def id_callback(self, feedback):
        item = feedback.menu_entry_id
        if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
            # unchecking something should lead to id 1
            self.robot_id = 1
            self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
            self.menu_handler.setCheckState(2, MenuHandler.CHECKED)
        else:
            items = [3, 4, 5, 6]
            self.robot_id = item - 2
            items.remove(item)
            for item_id in items:
                self.menu_handler.setCheckState(item_id, MenuHandler.UNCHECKED)
            self.menu_handler.setCheckState(item, MenuHandler.CHECKED)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

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
        self.pose.position.y = 1.0

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


class TeamMessage:
    def __init__(self, robot):
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
                    # beware, we use transforms3d here which has quaternion in different format than ros
                    robot_xyzw = self.robot.pose.orientation
                    robot_mat_in_world = quat2mat((robot_xyzw.w, robot_xyzw.x, robot_xyzw.y, robot_xyzw.z))
                    robot_trans_in_world = compose(
                        (self.robot.pose.position.x, self.robot.pose.position.y, self.robot.pose.position.z),
                        robot_mat_in_world, [1, 1, 1])
                    world_trans_in_robot = np.linalg.inv(robot_trans_in_world)
                    ball_xyzw = self.robot.ball.pose.orientation
                    mat_in_world = quat2mat((ball_xyzw.w, ball_xyzw.x, ball_xyzw.y, ball_xyzw.z))
                    trans_in_world = compose((self.robot.ball.pose.position.x, self.robot.ball.pose.position.y,
                                              self.robot.ball.pose.position.z),
                                             mat_in_world, [1, 1, 1])
                    trans_in_robot = np.matmul(world_trans_in_robot, trans_in_world)
                    pos_in_robot, mat_in_robot, _, _ = decompose(trans_in_robot)
                    quat_in_robot = mat2quat(mat_in_robot)

                    ball_relative = PoseWithCertainty()
                    ball_relative.pose.pose.position = Point(*pos_in_robot)
                    ball_relative.pose.pose.orientation = Quaternion(quat_in_robot[1], quat_in_robot[2],
                                                                     quat_in_robot[3], quat_in_robot[0])
                    ball_relative.confidence = self.robot.ball.confidence
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
