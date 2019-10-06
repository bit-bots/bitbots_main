import math
import rospy
import tf2_ros as tf2
from geometry_msgs.msg import PointStamped

from bitbots_head_behavior.actions.look_at import AbstractLookAt


class SearchRecentBall(AbstractLookAt):
    """
    This action looks at the last position the ball has been seen
    and starts searching it from this position on.
    """

    def __init__(self, dsd, blackboard, parameters=None):
        super(SearchRecentBall, self).__init__(dsd, blackboard, parameters)
        self.config = self.blackboard.config['search_recent_ball']

        self.pan_speed = self.config['pan_speed']
        self.tilt_speed = self.config['tilt_speed']

        self.offset_pattern = self.config['offset_pattern']

        self.threshold = self.blackboard.config['position_reached_threshold']

    def perform(self, reevaluate=False):
        """
        Call look_at to look at the point which our world-model determines to be the ball

        :param reevaluate: No effect here
        """
        # Get last ball position
        point = self.blackboard.world_model.get_ball_stamped()

        index, ball_id = self.blackboard.head_capsule.ball_specific_pattern_index

        if ball_id != hash(point):
            self.blackboard.head_capsule.ball_specific_pattern_index = (0, hash(point))
            index = 0

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

        current_head_pan += self.offset_pattern[index]
        current_head_tilt += self.offset_pattern[index]

        self.blackboard.head_capsule.send_motor_goals(current_head_pan, current_head_tilt, pan_speed=self.pan_speed, tilt_speed=self.tilt_speed)

        distance = math.sqrt((current_head_pan - head_pan) ** 2 + (current_head_tilt - head_tilt) ** 2)

        # Increment index when position is reached
        if distance < math.radians(self.threshold) and index < len(self.offset_pattern):
            self.blackboard.head_capsule.pattern_index = index + 1
