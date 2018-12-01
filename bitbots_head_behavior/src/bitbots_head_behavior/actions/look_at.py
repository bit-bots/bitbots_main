import rospy
import tf2_ros as tf2
from bio_ik_msgs.msg import IKRequest
from geometry_msgs.msg import PointStamped, Point

from bitbots_connector.blackboard import HeadBlackboard
from bitbots_dsd.abstract_action_element import AbstractActionElement


class AbstractLookAt(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        self.head_tf_frame = self.blackboard.config['head_transform_frame']
        self.tf_buffer = tf2.Buffer(rospy.Duration(5))
        self.bio_ik_request = IKRequest()

        # Service proxy for LookAt
        self.request = IKRequest()
        self.request.group_name = "Head"
        self.request.timeout.secs = 1
        self.request.attempts = 1
        self.request.approximate = True
        self.request.look_at_goals.append(LookAtGoal())
        self.request.look_at_goals[0].link_name = "head"
        self.request.look_at_goals[0].weight = 1
        self.request.look_at_goals[0].axis.x = 1

    def get_motor_goals_from_point(self, point):
        """Call the look at service to calculate head motor goals"""
        target = Point(point.x, point.y, point.z)
        self.request.look_at_goals[0].target = target
        response = self.blackboard.bio_ik(self.request).ik_response
        states = response.solution.joint_state
        return states.position[states.name.index('HeadPan')], states.position[states.name.index('HeadTilt')]

    def _look_at(self, point):
        """
        Look at a point which is relative to the robot.

        The points header.frame_id determines the transforms reference frame of this point
        :type point: PointStamped
        """
        # transform the points reference frame to be the head
        try:
            point = self.tf_buffer.transform(point, self.head_tf_frame)
        except tf2.LookupException as e:
            rospy.logerr('Could not find transform {}. Either it does not exist or '
                         'transform is not yet online.\n{}'.format(self.head_tf_frame, e))
            return
        except tf2.ConnectivityException as e:
            rospy.logerr('No connection to transform\n{}'.format(e))
            return

        head_pan, head_tilt = self.get_motor_goals_from_point(point)
        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt)


class LookDirection(AbstractLookAt):
    class Directions:
        FORWARD = 'FORWARD'
        DOWN = 'DOWN'
        UP = 'UP'

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        # Assert that a valid direction was supplied
        assert parameters is not None, 'No direction specified in parameters (key="direction")'
        assert 'direction' in parameters, 'No direction specified in parameters (key="direction")'
        assert parameters['direction'] in dir(self.Directions), 'Direction {} not found'.format(parameters["direction"])

        # Save supplied direction
        self.direction = getattr(self.Directions, parameters['direction'])

    def perform(self, reevaluate=False):
        self.publish_debug_data('direction', self.direction)

        # Construct target point from target direction
        point = PointStamped()
        point.header.frame_id = self.head_tf_frame
        point.point.x = 0
        point.point.y = 0
        if self.direction == self.Directions.DOWN:
            point.point.z = -10
        elif self.direction == self.Directions.UP:
            point.point.z = 10
        else:   # FORWARD
            point.point.z = 0

        # Call internal look-at to turn head to this point
        self._look_at(point)

