import math

import rospy
import tf2_ros as tf2
from bio_ik_msgs.msg import IKRequest, LookAtGoal
from geometry_msgs.msg import PointStamped, Point

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class AbstractLookAt(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractLookAt, self).__init__(blackboard, dsd, parameters)

        self.head_tf_frame = rospy.get_param('~base_link_frame', 'base_link')  # base_link is required by bio_ik
        self.camera_frame = rospy.get_param('~camera_frame', 'camera')
        self.bio_ik_request = IKRequest()

        # Service proxy for LookAt
        self.request = IKRequest()
        self.request.group_name = "Head"
        self.request.timeout.secs = 1
        self.request.approximate = True
        self.request.look_at_goals.append(LookAtGoal())
        self.request.look_at_goals[0].link_name = self.camera_frame
        self.request.look_at_goals[0].weight = 1
        self.request.look_at_goals[0].axis.x = 1

        self.pan_speed = self.blackboard.config['look_at']['pan_speed']
        self.tilt_speed = self.blackboard.config['look_at']['tilt_speed']


    def get_motor_goals_from_point(self, point):
        """Call the look at service to calculate head motor goals"""

        target = Point(point.x, point.y, point.z)
        self.request.look_at_goals[0].target = target
        response = self.blackboard.bio_ik(self.request).ik_response
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
            point = self.blackboard.head_capsule.tf_buffer.transform(point, self.head_tf_frame, timeout=rospy.Duration(0.9))
        except tf2.LookupException as e:
            rospy.logwarn('The frame {} is not being published (LookupException)'.format(self.head_tf_frame))
            return
        except tf2.ConnectivityException as e:
            rospy.logwarn('The transforms {} and {} are not connected in the TF Tree (ConnectivityException)'.format(point.header.frame_id, self.head_tf_frame))
            return
        except tf2.ExtrapolationException as e:
            rospy.logwarn('The transform {} is currently not available (ExtrapolationException)'.format(self.head_tf_frame))
            return

        head_pan, head_tilt = self.get_motor_goals_from_point(point.point)
        current_head_pan, current_head_tilt = self.blackboard.head_capsule.get_head_position()
        if abs(current_head_pan - head_pan) >= math.radians(min_pan_delta) or \
                abs(current_head_tilt - head_tilt) >= math.radians(min_tilt_delta):
            self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt, pan_speed=self.pan_speed, tilt_speed=self.tilt_speed)


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

        head_pan = head_pan / 180.0 * math.pi
        head_tilt = head_tilt / 180.0 * math.pi

        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt)
