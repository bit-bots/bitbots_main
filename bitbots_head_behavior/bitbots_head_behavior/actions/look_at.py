import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros as tf2
from bio_ik_msgs.msg import IKRequest, LookAtGoal
from geometry_msgs.msg import PointStamped, Point
from bitbots_moveit_bindings import get_bioik_ik

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class AbstractLookAt(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractLookAt, self).__init__(blackboard, dsd, parameters)

        self.head_tf_frame = self.blackboard.node.get_parameter('base_link_frame').value  # base_link is required by bio_ik
        self.camera_frame = self.blackboard.node.get_parameter('camera_frame').value
        self.bio_ik_request = IKRequest()

        # Service proxy for LookAt
        self.request = IKRequest()
        self.request.group_name = "Head"
        self.request.timeout.sec = 1
        self.request.approximate = True
        self.request.look_at_goals.append(LookAtGoal())
        # has to be without prefix since this is used for bio ik and frames inside the urdf are not prefixed
        self.request.look_at_goals[0].link_name = "camera"
        self.request.look_at_goals[0].weight = 1.0
        self.request.look_at_goals[0].axis.x = 1.0

        self.pan_speed = self.blackboard.config['look_at']['pan_speed']
        self.tilt_speed = self.blackboard.config['look_at']['tilt_speed']


    def get_motor_goals_from_point(self, point):
        """Call the look at service to calculate head motor goals"""
        self.blackboard.node.get_logger().warning("##########in get motor goals")
        target = Point(x=point.x, y=point.y, z=point.z)
        self.request.look_at_goals[0].target = target
        response = get_bioik_ik(self.request)
        states = response.solution.joint_state
        return states.position[states.name.index('HeadPan')], states.position[states.name.index('HeadTilt')]

    def look_at(self, point, min_pan_delta=0, min_tilt_delta=0):
        """
        Look at a point which is relative to the robot.

        The points header.frame_id determines the transforms reference frame of this point.
        The min_pan_delta and min_tilt_delta define minimal required movements in degrees to reduce unnecessary head movements.

        :type point: PointStamped
        :type min_pan_delta: float
        :type min_tilt_delta: float
        """
        # transform the points reference frame to be the head
        try:
            point = self.blackboard.head_capsule.tf_buffer.transform(point, self.head_tf_frame, timeout=Duration(seconds=0.9))
        except tf2.LookupException as e:
            self.blackboard.node.get_logger().warn('The frame {} is not being published (LookupException)'.format(self.head_tf_frame))
            return
        except tf2.ConnectivityException as e:
            self.blackboard.node.get_logger().warn('The transforms {} and {} are not connected in the TF Tree (ConnectivityException)'.format(point.header.frame_id, self.head_tf_frame))
            return
        except tf2.ExtrapolationException as e:
            self.blackboard.node.get_logger().warn('The transform {} is currently not available (ExtrapolationException)'.format(self.head_tf_frame))
            return

        head_pan, head_tilt = self.get_motor_goals_from_point(point.point)
        current_head_pan, current_head_tilt = self.blackboard.head_capsule.get_head_position()
        if abs(current_head_pan - head_pan) >= math.radians(min_pan_delta) or \
                abs(current_head_tilt - head_tilt) >= math.radians(min_tilt_delta):

            self.blackboard.head_capsule.send_motor_goals(head_pan,
                                                          head_tilt,
                                                          pan_speed=self.pan_speed,
                                                          tilt_speed=self.tilt_speed,
                                                          current_pan_position=current_head_pan,
                                                          current_tilt_position=current_head_tilt,
                                                          resolve_collision=True)



class LookDirection(AbstractLookAt):
    class Directions:
        """All possible directions"""
        FORWARD = 'FORWARD'
        DOWN = 'DOWN'
        UP = 'UP'

    def __init__(self, blackboard, dsd, parameters=None):
        """
        :param parameters['direction']: One of the possible directions
        """
        AbstractLookAt.__init__(self, blackboard, dsd, parameters)

        # Assert that a valid direction was supplied
        assert parameters is not None, 'No direction specified in parameters (key="direction")'
        assert 'direction' in parameters, 'No direction specified in parameters (key="direction")'
        assert parameters['direction'] in dir(self.Directions), 'Direction {} not found'.format(parameters["direction"])

        # Save supplied direction
        self.direction = getattr(self.Directions, parameters['direction'])
        self.position_down = self.blackboard.config['look_down_position']
        self.position_up = self.blackboard.config['look_up_position']
        self.position_forward = self.blackboard.config['look_forward_position']

    def perform(self, reevaluate=False):
        """
        Look at the direction direction that was supplied to this element

        :param reevaluate: No effect here
        """
        self.publish_debug_data('direction', self.direction)

        if self.direction == self.Directions.DOWN:
            head_pan, head_tilt = self.position_down
        elif self.direction == self.Directions.UP:
            head_pan, head_tilt = self.position_up
        else:
            head_pan, head_tilt = self.position_forward

        head_pan = math.radians(head_pan)
        head_tilt = math.radians(head_tilt)

        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt)
